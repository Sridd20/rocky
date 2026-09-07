# Rocky — Pentapod Robot

> 5-legged robot with dual Arduino brain, MuJoCo-validated 3+2 sequential gait,
> and a stationary manipulation mode using a 3-leg tripod base + 2 forward-facing arm legs.

---

## Overview

Rocky is a pentapod (5-legged) robot built around an unconventional weight-balancing strategy:

- **3 legs** form a stable support triangle at all times
- **2 legs** (placed forward-facing) either step forward (walk mode) or act as manipulator arms (stationary mode)

The gait algorithm is based on the Liu et al. *Five-Limbed Robot* sequential stability analysis paper, with a key modification: **the 2 swing legs face forward instead of the 3**, shifting the robot's active manipulation capability to the front.

---

## Project Phases

### ✅ Phase 1 — Hardware Design
- **Asymmetric pentagon body** — elongated bottom sides (14.87 cm × 2) offset CoM toward tripod legs
  - Top edge: 10.653 cm | Right-upper/Left-upper sides: 15.392 cm | Bottom sides: 14.87 cm × 2
  - Geometric centroid sits 15.8 mm above true center → biases effective CoM toward legs 2, 3, 4
- 3 DOF per leg: **Hip** (yaw Z), **Knee** (pitch Y), **Ankle** (pitch Y)
- 15 total servo motors
- Dual Arduino UNO controller architecture (brain split across two boards)

### ✅ Phase 2 — MuJoCo Simulation Setup
- Full MJCF model in [`pentapod.xml`](pentapod.xml)
- Freejoint base, 5 legs with correct link geometry
  - Coxa: 45 mm | Femur: 90 mm + 40 mm offset | Tibia: 60 mm + 90 mm
- Touch sensors on each foot
- Position actuators (kp=5) for all 15 joints
- Interactive viewer via [`view_pentapod.py`](view_pentapod.py)

### ✅ Phase 3 — Stability Analysis
- [`stability_analysis.py`](stability_analysis.py) evaluates all 120 gait permutations
- Replicates Liu et al. methodology: support polygon margin, CoM projection
- Identifies optimal swing-pair sequence for given walk direction

### ✅ Phase 4 — Gait Controller (In Progress)
- [`gait_runner.py`](gait_runner.py) — main locomotion controller
- **Stationary mode:** 3-leg tripod (legs 1, 3, 4) + 2 raised arm legs (legs 0, 2)
- **Walk mode:** Sequential 3+2 gait with sinusoidal lift arc
  - Swing pair computed per direction using exact hip-angle geometry
  - Support legs hold neutral stance (stable, no IK drift)
  - `--speed` flag for slow-motion inspection

**Key bugs fixed so far:**
| Bug | Fix |
|-----|-----|
| MuJoCo position actuators expect **radians**, not degrees | `set_ctrl` now converts deg→rad |
| `LIFT_KNEE=+15°` drove feet 7cm underground | Corrected to `LIFT_KNEE=-55°` (negative = femur rises) |
| IK support tracking cascaded body tilt into collapse | Replaced with neutral-stance support (no IK) |
| Freejoint initial height unreliable | Explicit `data.qpos[addr:] = [0,0,0.25,1,0,0,0]` |

### 🔲 Phase 5 — Hardware Integration
- Export gait joint angle sequences to Arduino-compatible format
- Map MuJoCo actuator targets → servo PWM signals
- Tune stance/lift angles on physical hardware

### 🔲 Phase 6 — Closed-Loop Control
- IMU feedback for body tilt correction
- Foot contact sensing for adaptive gait timing
- Direction control via RC or autonomous path planning

---

## Gait Design

```
Top view — asymmetric body, legs numbered at pentagon vertices:

             [L0]─────[L1]        ← top edge  (10.65 cm, SHORT)
            /   115°  65°  \
          /                  \    ← upper sides (15.39 cm each)
        /                      \
     [L4]         ★CoM         [L2]   ← widest point
    -157°    (biased toward     -23°
        \    L2/L3/L4 side)   /
          \                 /   ← bottom sides (14.87 cm each)
            \             /
             ──────[L3]──         ← bottom vertex (-90°, LONG)

★ CoM sits inside the L2-L3-L4 support triangle at all times.
  Legs 0 & 1 (top) are the SWING / ARM pair.
  Legs 2, 3, 4 (bottom) are the stable TRIPOD.
```

**5-phase cycle** (each phase lifts 2 legs simultaneously):

| Phase | Swing | Support (tripod) |
|-------|-------|------------------|
| 1 | 0, 1 | 2, 3, 4 |
| 2 | 0, 2 | 1, 3, 4 |
| 3 | 1, 3 | 0, 2, 4 |
| 4 | 0, 4 | 1, 2, 3 |
| 5 | 1, 2 | 0, 3, 4 |

> **Note:** Legs 0 & 1 are the primary swing pair since the CoM is already biased
> toward legs 2, 3, 4. When either 0 or 1 is lifted, the CoM remains inside the
> support polygon formed by the remaining four legs.

---

## Quick Start

```bash
# Install MuJoCo Python bindings
pip install mujoco

# View static stance
python view_pentapod.py --static

# Free-fall test (robot drops onto legs)
python view_pentapod.py

# Run gait simulation
python gait_runner.py --mode walk --direction 0

# Slow motion for inspection (5x slower)
python gait_runner.py --mode walk --direction 0 --speed 0.2

# Stationary / arm mode
python gait_runner.py --mode stationary
```

---

## File Structure

```
rocky/
├── pentapod.xml          # MuJoCo MJCF robot model
├── view_pentapod.py      # Static viewer + free-fall test
├── gait_runner.py        # Gait controller (walk + stationary)
├── stability_analysis.py # Gait permutation stability evaluator
├── .gitignore
└── docs/
    └── Five-limbed_robot_R10.md   # Reference paper (Liu et al.)
```

---

## Current Status

**Phase 4 — active development**

The simulation runs stably for hundreds of gait cycles. The robot maintains its stance
height and the 5-phase gait sequence executes without collapse. Forward locomotion from
hip angle stepping is under tuning — the geometry is correct, net displacement per cycle
is being verified visually at `--speed 0.2`.

Next: confirm visible forward travel in simulation, then begin Phase 5 hardware mapping.
