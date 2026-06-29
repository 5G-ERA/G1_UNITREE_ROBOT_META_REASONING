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


import statistics as _st


class SensingMonitor:
    """The robot's feedback on its OWN sensing capacity (Renxi: 'otherwise we are purely guessing').

    From real, live signals it estimates how much to trust perception right now:
      laser_noise (0..1) — frame-to-frame instability of the LiDAR scan: short-window std of the
                           forward clearance + variability of the live point count + scan churn.
      loc_conf    (0..1) — localisation confidence (scan-to-map match; the firmware gives no covariance).
      reliability (0..1) — combined self-assessed sensing capacity = loc_conf * (1 - laser_noise),
                           further reduced by recent relocalisation jumps.
    This is the paper's 'distributed plausibility / sensing reliability' meta-parameter, grounded
    in measured noise rather than assumed.
    """

    def __init__(self, win=1.5, c0_ref=0.40, hist_cap=600):
        self.win = win              # s window for the noise estimate
        self.c0_ref = c0_ref        # m of clearance std that counts as 'fully noisy'
        self._c0 = deque(); self._n = deque(); self._jumps = deque()
        self._prev_cells = None
        self.hist = deque(maxlen=hist_cap)   # (t, reliability, laser_noise, loc_conf)

    def update(self, now, live_cells, c0_m, loc_match, reloc_jump=False):
        live_cells = set(live_cells) if live_cells else set()
        self._c0.append((now, c0_m))
        while self._c0 and now - self._c0[0][0] > self.win:
            self._c0.popleft()
        c0v = [v for _, v in self._c0]
        c0_std = _st.pstdev(c0v) if len(c0v) >= 2 else 0.0

        self._n.append((now, len(live_cells)))
        while self._n and now - self._n[0][0] > self.win:
            self._n.popleft()
        nv = [v for _, v in self._n]
        n_mean = (sum(nv) / len(nv)) if nv else 0.0
        n_std = _st.pstdev(nv) if len(nv) >= 2 else 0.0
        n_cv = (n_std / n_mean) if n_mean > 0 else 0.0

        churn = 0.0
        if self._prev_cells is not None and (live_cells or self._prev_cells):
            uni = len(live_cells | self._prev_cells)
            inter = len(live_cells & self._prev_cells)
            churn = 1.0 - (inter / uni if uni else 1.0)
        self._prev_cells = live_cells

        if reloc_jump:
            self._jumps.append(now)
        while self._jumps and now - self._jumps[0] > 10.0:
            self._jumps.popleft()
        jrate = len(self._jumps)

        laser_noise = clip01(0.6 * (c0_std / self.c0_ref) + 0.4 * min(1.0, n_cv * 2.0))
        loc_conf = clip01(loc_match if loc_match is not None else 1.0)
        reliability = clip01(loc_conf * (1.0 - laser_noise))
        if jrate > 0:
            reliability *= max(0.0, 1.0 - 0.3 * jrate)

        self.hist.append((round(now, 2), round(reliability, 3), round(laser_noise, 3), round(loc_conf, 3)))
        return {"reliability": round(reliability, 3), "laser_noise": round(laser_noise, 3),
                "loc_conf": round(loc_conf, 3), "c0_std": round(c0_std, 3),
                "nobs_cv": round(n_cv, 3), "scan_churn": round(churn, 3), "reloc_rate10s": jrate}

    def history(self):
        return list(self.hist)
