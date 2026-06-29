# The two robot metrics: clearance + progression

Per Renxi's request — extract **two separate metrics from the robot, clearance and
progression, and show them together** so we get a meaningful picture of the robot's
perception and performance instead of blind testing. These are the first two
meta-parameters of the paper's Shared Experience Interface.

## Definitions

| Metric | Meaning | Range | Source |
|---|---|---|---|
| **clearance** | how much free space is ahead (perception) | 0..1 | forward clearance `c0` (m) from LiDAR + depth, normalised by 1.5 m |
| **progression** | how fast the robot is actually reaching the goal (performance) | 0..1 | rate of decrease of distance-to-goal over a 2 s window, normalised by 0.30 m/s |

`clearance = 1.0` → ≥ 1.5 m open ahead; `0` → blocked.
`progression = 1.0` → moving toward the goal at full nominal speed; `0` → stalled or
moving away. (The paper's `clearance_pressure` = `1 - clearance`.)

Both are computed in `g1_metrics.py` (`SEIMetrics.update(now, d_goal, c0)`), normalised
and independent, so they can be read on one 0..1 axis.

## Where they appear

- **Logged every tick** in each `dataset/*.json` sample: `clearance`, `clearance_m`,
  `progression`, `progress_rate` (alongside pose, c0, battery, loc_match…).
- **Console**, each line: `... clear=0.42 prog=0.18 ...`.
- **Live window** (`gotoviz`): a fourth panel (bottom-right) plots clearance and
  progression over the last ~40 s, with the current values in its title.

## Visualize a run

```bash
python plot_metrics.py dataset/<run>.json        # or no arg = newest run
```

Produces `<run>_metrics.png`:
- **top:** clearance and progression vs time, collisions marked.
- **bottom:** the path coloured by clearance (green = open, red = blocked), collisions
  as black ×.

It also prints a one-line summary, e.g. for the first successful door crossing:

```
mean clearance=0.43  mean progression=0.20  ticks blocked(clear<0.2)=295  ticks stalled(prog<0.1)=541
```

i.e. the robot spent a large fraction of the run blocked and stalled at the doorway —
visible at a glance instead of guessed. That is the baseline the GPU-vision upgrade is
meant to improve: with the table seen by depth, the red/stalled band at the door should
shrink (higher mean clearance and progression, fewer blocked/stalled ticks).

## Tuning (if needed)

In `g1_metrics.py`, `SEIMetrics(clear_full=1.5, prog_ref=0.30, prog_win=2.0)`:
- `clear_full`: metres of clearance counted as "fully clear".
- `prog_ref`: m/s that counts as full progression (≈ nominal walking speed).
- `prog_win`: seconds over which the progress rate is averaged (smaller = twitchier).
