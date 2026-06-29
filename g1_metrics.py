"""
g1_metrics.py — the first two Shared-Experience metrics the tutor asked for.

Renxi: "extract two separate metrics, one for CLEARANCE and one for PROGRESSION,
so we get a meaningful visualization of the robot's perception and performance
rather than blind testing."

These are the first two meta-parameters of the paper's Shared Experience Interface:

  CLEARANCE  (perception)  — how much free space is ahead of the robot, 0..1.
                             1.0 = open (>= CLEAR_FULL metres clear), 0 = blocked.
                             Source: forward clearance c0 (metres) from LiDAR (+depth).
                             Paper's "clearance_pressure" = 1 - clearance.

  PROGRESSION (performance) — is the robot actually getting to the goal, 0..1.
                             Rate at which distance-to-goal shrinks, normalised by a
                             nominal walking speed. 1.0 = moving to the goal at full
                             speed, 0 = stalled or moving away.

Both are normalised and independent, so plotting them together is informative:
e.g. clearance -> 0.1 AND progression -> 0 at the same time = "stuck at the door",
which is exactly what blind testing cannot show.
"""
from __future__ import annotations
from collections import deque


def clip01(v):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return 0.0
    return 0.0 if v < 0 else (1.0 if v > 1 else v)


class SEIMetrics:
    """Per-tick clearance + progression. Stateful (keeps a short window for the rate)."""

    def __init__(self, clear_full=1.5, prog_ref=0.30, prog_win=2.0, hist_cap=600):
        self.clear_full = clear_full      # m of forward clearance counted as 'fully clear'
        self.prog_ref = prog_ref          # m/s that counts as 'full progression'
        self.prog_win = prog_win          # s window used to estimate the progress rate
        self._dwin = deque()              # (t, d_goal) for the rate estimate
        self.hist = deque(maxlen=hist_cap)  # (t, clearance, progression) for plotting

    def update(self, now, d_goal, c0_m):
        """now=seconds, d_goal=distance to goal (m), c0_m=forward clearance (m).
        Returns {clearance, clearance_m, progression, progress_rate}."""
        clearance = clip01(c0_m / self.clear_full) if self.clear_full > 0 else 0.0

        self._dwin.append((now, d_goal))
        while len(self._dwin) >= 2 and now - self._dwin[0][0] > self.prog_win:
            self._dwin.popleft()
        progress_rate = 0.0
        if len(self._dwin) >= 2:
            t0, d0 = self._dwin[0]
            dt = now - t0
            if dt > 1e-3:
                progress_rate = (d0 - d_goal) / dt          # m/s toward the goal (neg = away)
        progression = clip01(progress_rate / self.prog_ref) if self.prog_ref > 0 else 0.0

        self.hist.append((round(now, 2), round(clearance, 3), round(progression, 3)))
        return {"clearance": round(clearance, 3), "clearance_m": round(c0_m, 2),
                "progression": round(progression, 3), "progress_rate": round(progress_rate, 3)}

    def history(self):
        """List of (t, clearance, progression) for the live strip chart / export."""
        return list(self.hist)
