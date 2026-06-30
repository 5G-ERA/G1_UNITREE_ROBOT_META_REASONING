"""
g1_fsm.py — the paper's PRIMARY physical baseline: a Reactive Threshold-Based FSM / Behavior Tree.

It consumes the SAME observable streams as the meta-reasoner (the Shared Experience Interface:
clearance, progression, payload/spill, human proximity, battery) but governs behaviour with FIXED
thresholds -> fixed transitions (slow / stop / fallback). No QoE analogies, no deployability tension,
no distributed plausibility, no stability filtering, no insufficiency reasoning. This is the
"realistic deployable alternative" the paper (Sec. VII.E) compares DCE against, over identical inputs.

Mirror of g1_meta.MetaController so g1_goto can wire either one the same way (opt-in G1_FSM=1).
Difference is only the reasoning architecture: hard thresholded transitions vs experience-scoped
governance.
"""
from __future__ import annotations

# alarm-zone thresholds (normalised 0..1 unless noted) — tune once, then FREEZE for the trials
CLEAR_SLOW = 0.50      # clearance below this -> slow down
CLEAR_STOP = 0.25      # clearance below this -> reactive stop
PAYLOAD_LOW = 0.30     # payload (cup) below this (e.g. a spill) -> slow down
HUMAN_STOP = True      # a human detected near -> stop and wait
BATT_LOW = 20.0        # battery % below this -> conservative return speed

FWD_NORMAL = 0.45
FWD_CAUTIOUS = 0.38
FWD_SLOW = 0.33


class FSMBaseline:
    """Reactive threshold FSM. step() returns the state + control, same shape as MetaController."""

    def __init__(self):
        self.state = "NORMAL"
        self.prev_state = "NORMAL"
        self.interventions = 0     # times it left NORMAL into a reactive state (the FSM's "alarms")
        self.transitions = 0       # any state change (for chatter comparison)

    def step(self, clearance, progression, payload, human_near=False, battery=None):
        # priority order = a fixed behaviour tree (safety first, then payload, then clutter, then power)
        if human_near and HUMAN_STOP:
            state, fwd, stop = "STOP_HUMAN", 0.0, True
        elif clearance is not None and clearance < CLEAR_STOP:
            state, fwd, stop = "STOP_CLEAR", 0.0, True
        elif payload is not None and payload < PAYLOAD_LOW:
            state, fwd, stop = "SLOW_PAYLOAD", FWD_SLOW, False
        elif clearance is not None and clearance < CLEAR_SLOW:
            state, fwd, stop = "CAUTIOUS", FWD_CAUTIOUS, False
        elif battery is not None and battery < BATT_LOW:
            state, fwd, stop = "RETURN_BATT", FWD_CAUTIOUS, False
        else:
            state, fwd, stop = "NORMAL", FWD_NORMAL, False

        if state != self.state:
            self.transitions += 1
            if state != "NORMAL":
                self.interventions += 1
        self.prev_state, self.state = self.state, state
        return {"state": state, "fwd": fwd, "stop": stop,
                "intervention": (state != "NORMAL" and self.prev_state == "NORMAL"),
                "interventions": self.interventions, "transitions": self.transitions}


# --------- self-test ---------
if __name__ == "__main__":
    f = FSMBaseline()
    seq = [
        ("open",      dict(clearance=0.9, progression=0.6, payload=0.9)),
        ("narrow",    dict(clearance=0.4, progression=0.5, payload=0.9)),       # -> CAUTIOUS
        ("spill",     dict(clearance=0.6, progression=0.5, payload=0.08)),      # -> SLOW_PAYLOAD
        ("human",     dict(clearance=0.6, progression=0.4, payload=0.9, human_near=True)),  # -> STOP_HUMAN
        ("tight",     dict(clearance=0.15, progression=0.2, payload=0.9)),      # -> STOP_CLEAR
        ("open2",     dict(clearance=0.9, progression=0.6, payload=0.9)),       # -> NORMAL
    ]
    for name, kw in seq:
        d = f.step(**kw)
        print(f"{name:7s} -> {d['state']:13s} fwd={d['fwd']} stop={d['stop']}")
    print("interventions:", f.interventions, "transitions:", f.transitions)
    assert f.interventions >= 4
    print("OK")
