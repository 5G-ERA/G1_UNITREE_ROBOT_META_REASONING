"""
g1_meta.py — closed-loop meta-reasoning for the G1, wiring the supervisor's ESCMR reasoner
(escmr_reasoner.py = the runtime form of Chapter 5 "Analogy for meta-reasoning") to the live robot.

Idea (thesis Ch.5): the robot governs WHICH navigation analogy frames its behaviour from the
produced experience. Here the experience has three shared meta-parameters (all 0..1, 1 = good):

    safety       — free space / no contact ahead  (from clearance; drops on collision)
    progression  — actually advancing to the goal
    payload      — cup stability (1 = steady; DROPS hard when the human marks a spill)

Analogy library (each = a behaviour with its own control params + QoE expectations):
    efficient_nav     — fast, low caution. Expects HIGH payload -> a spill makes it ALERT (vetoed).
    cautious_nav      — slower, more clearance. Tolerates moderate payload loss.
    payload_sensitive — slowest, smoothest. Calibrated to operate when payload is poor (a spill) ->
                        it is the analogy that survives a spill, so the reasoner SWITCHES to it.

So: human marks a spill -> payload meta-parameter collapses -> efficient_nav goes into hard alert and
is vetoed -> the reasoner switches the governing analogy to payload_sensitive -> the robot slows and
smooths for the rest of the run. That is the keep->switch governance of the paper, driven by the
spill ground truth.

Opt-in: g1_goto reads G1_META=1 to enable this. Default OFF = current behaviour untouched.
"""
from __future__ import annotations
from escmr_reasoner import (
    ExperienceScopedCapabilityMetaReasoner, ReasonerCalibration, ReasonerRuntimeConfig,
    AnalogyQoESpecification, FourSemanticRegionCalibration, SensorCalibration, ReasonerFeedbackAction,
)

META = ("safety", "progression", "payload")

# Per-analogy control parameters applied to the robot when that analogy governs.
#   fwd      = forward speed target (m/s) for the DWA (stays > deadzone ~0.3)
#   robot_r  = clearance/safety radius used by the DWA (bigger = more cautious)
CONTROL = {
    "efficient_nav":     {"fwd": 0.45, "robot_r": 0.20},
    "cautious_nav":      {"fwd": 0.38, "robot_r": 0.26},
    "payload_sensitive": {"fwd": 0.33, "robot_r": 0.30},
}


def _cal(exp, adp, dng):
    return FourSemanticRegionCalibration(exp, adp, dng)


def build_reasoner(ablation=None):
    # region calibration boundaries are on the 0..1 signal (higher = better experience)
    efficient = AnalogyQoESpecification(
        "efficient_nav", META,
        {"safety": _cal(0.55, 0.35, 0.18), "progression": _cal(0.45, 0.20, 0.08),
         "payload": _cal(0.70, 0.45, 0.30)},                       # STRICT payload -> a spill (low payload) = dangerous -> ALERT
        {"safety": 0.30, "progression": 0.55, "payload": 0.15},
        {"safety": 1.0, "progression": 0.0, "payload": 0.6})
    cautious = AnalogyQoESpecification(
        "cautious_nav", META,
        {"safety": _cal(0.70, 0.45, 0.22), "progression": _cal(0.40, 0.18, 0.05),
         "payload": _cal(0.50, 0.30, 0.12)},
        {"safety": 0.55, "progression": 0.25, "payload": 0.20},
        {"safety": 1.0, "progression": 0.0, "payload": 0.5})
    payload = AnalogyQoESpecification(
        "payload_sensitive", META,
        {"safety": _cal(0.60, 0.35, 0.18), "progression": _cal(0.35, 0.12, 0.04),
         "payload": _cal(0.30, 0.12, 0.02)},                       # LENIENT payload -> survives a spill (no alert)
        {"safety": 0.35, "progression": 0.15, "payload": 0.50},
        {"safety": 1.0, "progression": 0.0, "payload": 1.0})       # payload non-compensable here
    # --- ablation knobs (paper Sec. VII.I diagnostic variants) ---
    mem = 0.001 if ablation == "instantaneous" else 1.0       # tiny memory window = no temporal persistence (1 tick)
    hyst = 0.0 if ablation == "no_hysteresis" else 0.05        # remove the dynamic switching margin
    ss = 0.0 if ablation == "no_plausibility" else None        # 0 reliability-sensitivity = no distributed plausibility
    sc = {"safety": SensorCalibration(0.95, 0.10 if ss is None else ss),
          "progression": SensorCalibration(1.0, 0.08 if ss is None else ss),
          "payload": SensorCalibration(0.9, 0.15 if ss is None else ss)}
    cal = ReasonerCalibration(
        (efficient, cautious, payload), sc,
        {"efficient_nav": 0.6, "cautious_nav": 0.25, "payload_sensitive": 0.15},
        initial_active_analogy="efficient_nav",
        runtime_config=ReasonerRuntimeConfig(frequency_hz=1.0, memory_duration_minutes=mem,
                                             hysteresis_margin=hyst))
    return ExperienceScopedCapabilityMetaReasoner.from_calibration(cal)


class MetaController:
    """Thin wrapper: feed it the live experience, it returns the governing analogy + control params."""

    # ablation: None (full DCE) | "instantaneous" | "no_hysteresis" | "no_plausibility" | "no_insufficiency"
    def __init__(self, ablation=None):
        self.ablation = ablation
        self.r = build_reasoner(ablation)
        self.active = self.r.active_analogy
        self.action = "KEEP_CURRENT_ANALOGY"
        self.explanation = ""

    def step(self, safety, progression, payload, timestamp=None):
        payload = max(0.0, min(1.0, float(payload)))
        # task attention (thesis: the task's attention vector). Coffee delivery cares MORE about payload
        # as it degrades -> when the cup spills, payload attention rises and pulls governance toward the
        # payload-sensitive analogy; nominally it stays low so the robot runs efficient.
        task_attention = {"safety": 0.5, "progression": 0.5, "payload": 0.30 + 0.60 * (1.0 - payload)}
        fb = self.r.evaluate_latest_reading(
            {"safety": float(safety), "progression": float(progression), "payload": payload},
            task_attention=task_attention, timestamp=timestamp)
        active = fb.active_after
        action = fb.recommended_action.value if hasattr(fb.recommended_action, "value") else str(fb.recommended_action)
        switched = fb.recommended_action == ReasonerFeedbackAction.SWITCH_ANALOGY
        if self.ablation == "no_insufficiency" and fb.terminate:
            # ablation: never declare insufficiency/fallback -> FORCE the highest-comfort analogy
            best = max(fb.analogy_evaluations.values(), key=lambda e: e.comfort_score)
            if best.analogy_name != active:
                switched = True
            active = best.analogy_name; action = "FORCED_SELECT"; self.r.active_analogy = active
        self.active = active; self.action = action; self.explanation = fb.explanation
        ctl = CONTROL.get(self.active, CONTROL["cautious_nav"])
        return {"active": self.active, "action": self.action,
                "fwd": ctl["fwd"], "robot_r": ctl["robot_r"], "switched": switched,
                "terminate": (fb.terminate and self.ablation != "no_insufficiency"), "explanation": fb.explanation}


# --------- self-test: spill should switch efficient_nav -> payload_sensitive ---------
if __name__ == "__main__":
    m = MetaController()
    print("start:", m.active)
    for i in range(6):                      # nominal: clear, advancing, cup steady
        d = m.step(0.85, 0.6, 0.9, timestamp=i)
    print("nominal ->", d["active"], "(fwd", d["fwd"], ")")
    for i in range(6):                      # SPILL: payload collapses
        d = m.step(0.85, 0.5, 0.08, timestamp=10 + i)
    print("after spill ->", d["active"], "(fwd", d["fwd"], ") action=", d["action"])
    assert d["active"] == "payload_sensitive", "expected switch to payload_sensitive after spill"
    print("OK: spill -> switched to payload_sensitive (slower/smoother)")
