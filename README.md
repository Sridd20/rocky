# Rocky — Pentapod Robot

> 5-legged walking robot with asymmetric pentagon body, MuJoCo-validated 3+2 sequential gait,
> and a stationary manipulation mode using a 3-leg tripod base + 2 forward-facing arm legs.
> Designed for irregular terrain traversal using active CoM balancing across a 5-phase gait cycle.

---

## Overview

Rocky is a pentapod (5-legged) robot built around an unconventional weight-balancing strategy:

- **3 legs** (L2, L3, L4) form a stable support triangle at all times — the **tripod**
- **2 legs** (L0, L1) either step forward (walk mode) or act as manipulator arms (stationary mode) — the **swing pair**
- An **asymmetric pentagon body** geometrically biases the frame toward the tripod side
- **Active hip balancing** compensates for the CoM shift caused by heavy MG996R knee/ankle servos

The gait algorithm is based on the Liu et al. *Five-Limbed Robot* sequential stability analysis,
with the swing pair facing forward rather than the tripod, enabling simultaneous locomotion and manipulation.

---

## Hardware

### Servo Configuration (Mixed Build)

| Joint | Servo | Qty | Mass each | Torque | Notes |
|---|---|---|---|---|---|
| **Hip** (yaw Z) | SG90 | 5 | 9g | 1.8 kg·cm | Horizontal swing, low load |
| **Knee** (pitch Y) | MG996R | 5 | 55g | 9.4 kg·cm | Main load-bearing, metal gears |
| **Ankle** (pitch Y) | MG996R | 5 | 55g | 9.4 kg·cm | Ground reaction, metal gears |

SG90 is sufficient for the hip (horizontal swing, ~1.0 kg·cm needed).
MG996R is required for knee/ankle — on irregular terrain the dynamic load reaches ~3 kg·cm,
which exceeds SG90's 1.8 kg·cm limit and would strip plastic gears.

### Electronics

| Component | Mass | Notes |
|---|---|---|
| Arduino Uno Q | 32g | 68.58 × 53.34 mm, 30–33.5g |
| PCA9685 16-ch servo driver | 5g | Required — Uno has only 6 PWM pins |
| 2S 500mAh LiPo | 28g | 2S needed for MG996R stall current |
| 5V BEC regulator | 5g | Logic power |
| Wiring + connectors | ~8g | |

### Mass Budget

| Group | Mass |
|---|---|
| Body (plate + electronics + battery) | 83g |
| 5 × leg set (coxa+femur+tibia shells) | 176g |
| 5 × SG90 hip + brackets | 58g |
| 10 × MG996R knee/ankle + brackets | 595g |
| **Total** | **~908g** |

### Key Geometry

| Measurement | Value |
|---|---|
| L0–L1 hip span | 87.8 mm |
| L0–L1 foot span (neutral) | 252.6 mm |
| L0/L1 → nearest tripod (L4/L2) | 396.8 mm |
| L0/L1 → L3 (furthest) | 609.6 mm |
| Tripod L2–L4 top edge Y | −105.8 mm |
| Foot reach from centre (L3 dir) | 325 mm |

---

## Centre of Mass Analysis

The asymmetric pentagon's **geometric centroid is only −2.5 mm Y** — barely off centre.
The short top edge (L0–L1) vs elongated bottom sides nearly balance out.

With the mixed servo build, **MG996R legs dominate the mass** (821g legs vs 83g body):

```
Pentagon geometric centroid :   -2.5 mm Y   (shape alone gives almost nothing)
Body inertial CoM (XML)     : -120.0 mm Y   (battery pushed toward L3)
Full system CoM (body+legs) :   ~-7.0 mm Y  (leg mass nearly cancels bias)
Tripod L2-L4 support edge   : -105.8 mm Y   (CoM must be below this for passive balance)
```

**The system CoM (−7mm) is outside the tripod support triangle by ~99mm.**
This means the robot cannot passively balance on 3 legs — it requires **active hip actuation**
to shift effective CoM during stance transitions. This is intentional and analogous to how
biological walkers dynamically shift weight.

**Physical build rule:** mount the LiPo as far toward L3 (rear) as possible (~120–130mm from centre).

---

## Body Layout (Top View)

```
              [L0]────[L1]         ← top edge  (87.8mm hip span, 252.6mm foot span)
             /  115°   65°  \
           /                  \    ← upper sides
         /                      \
      [L4]          ★           [L2]   ← widest point
     -157°    (CoM target)      -23°
         \    -120mm Y  /
           \           /           ← lower sides (elongated)
             \       /
              ──[L3]──             ← bottom vertex (-90°, furthest reach 325mm)

  SWING  = L0, L1  (top, SG90 hip + MG996R knee/ankle)
  TRIPOD = L2, L3, L4  (bottom, same servo config)
  ★ CoM = -7mm Y (active balancing required to stay inside tripod triangle)
```

---

## Gait Design

**5-phase cycle** — each phase lifts 2 non-adjacent legs simultaneously:

| Phase | Swing pair | Support tripod |
|---|---|---|
| 1 | L0, L1 | L2, L3, L4 |
| 2 | L0, L2 | L1, L3, L4 |
| 3 | L1, L3 | L0, L2, L4 |
| 4 | L0, L4 | L1, L2, L3 |
| 5 | L1, L2 | L0, L3, L4 |

The gait sequence order is computed per walking direction by `build_gait_sequence()` in
[`gait_runner.py`](gait_runner.py) — the pair most aligned with the walk direction steps first.

---

## Simulation

### Setup

```bash
pip install mujoco
```

### Run

```bash
# View static stance
python view_pentapod.py --static

# Free-fall drop test (robot settles onto legs)
python view_pentapod.py

# Walk forward
python gait_runner.py --mode walk --direction 0

# Walk in any direction (degrees)
python gait_runner.py --mode walk --direction 90

# Slow motion — recommended for inspection (5x slower)
python gait_runner.py --mode walk --direction 0 --speed 0.2

# Stationary mode: tripod base + 2 raised arm legs
python gait_runner.py --mode stationary

# Stability analysis — evaluates all 120 gait permutations
python stability_analysis.py

# Leg geometry analysis + top-down diagram
python leg_geometry_analysis.py
```

### Simulation Parameters

| Parameter | Value | Notes |
|---|---|---|
| Total simulated mass | 908g | Matches physical build |
| Body mass | 83g | Plate + Uno Q + PCA9685 + LiPo + BEC |
| Coxa mass | 19g | Shell + SG90 + bracket |
| Femur mass | 74g | Shell + MG996R + bracket |
| Tibia mass | 72g | Shell + MG996R + bracket |
| Timestep | 2ms | `option timestep="0.002"` |
| Position kp | 5 | May need tuning for 908g robot |
| Force range | ±2 N | May need increase for MG996R torque |

---

## File Structure

```
rocky/
├── pentapod.xml              # MuJoCo MJCF model (masses, geometry, sensors)
├── gait_runner.py            # Gait controller — walk + stationary modes
├── stability_analysis.py     # All 120 gait permutation stability evaluator
├── leg_geometry_analysis.py  # Geometry + CoM analysis, generates diagram
├── leg_geometry.png          # Top-down leg geometry diagram
├── view_pentapod.py          # Static viewer + free-fall test
├── stability_results.csv     # Full 120×10 stability margin matrix
├── top_gaits.txt             # Top-ranked stable gait sequences
├── .gitignore
└── docs/
    └── Five-limbed_robot_R10.md   # Reference: Liu et al. gait paper
```

---

## Project Phases

| Phase | Status | Description |
|---|---|---|
| 1 — Hardware Design | ✅ | Asymmetric pentagon body, 3-DOF legs, servo selection |
| 2 — MuJoCo Model | ✅ | Full MJCF, realistic masses, touch sensors, freejoint |
| 3 — Stability Analysis | ✅ | 120 permutation sweep, optimal gait sequence identified |
| 4 — Gait Controller | 🔄 | Walk + stationary running, forward travel under tuning |
| 5 — Hardware Integration | 🔲 | Joint angles → Arduino PWM, physical servo tuning |
| 6 — Closed-Loop Control | 🔲 | IMU tilt feedback, foot contact adaptive timing |

### Key Bugs Fixed

| Bug | Fix |
|---|---|
| Position actuators expected radians | `set_ctrl()` now converts deg→rad |
| `LIFT_KNEE=+15°` drove feet 7cm underground | Corrected to `LIFT_KNEE=-55°` |
| IK support tracking cascaded into collapse | Replaced with neutral-stance hold |
| Freejoint initial height unreliable | Explicit `qpos[addr:] = [0,0,0.25,1,0,0,0]` |
| Body CoM (−8mm) outside tripod triangle | Set to −120mm Y (battery placement) |
| Placeholder masses (1225g) | Updated to realistic 908g mixed build |

---

## Current Status

**Phase 4 — active development**

Simulation runs stably for hundreds of gait cycles. Mass model updated to reflect the physical
mixed servo build (SG90 hip, MG996R knee/ankle, Arduino Uno Q, 2S LiPo — ~908g total).
Geometry analysis confirms L0–L1 foot span of 252.6mm and 396.8mm cross-diagonals for
gait transition overlap. Active CoM balancing via hip servos is required during 3-leg stance
due to MG996R leg mass (821g) dominating the 83g body.

Next: tune `kp` and `forcerange` for 908g robot, confirm visible forward travel at `--speed 0.2`,
then begin Phase 5 Arduino PWM mapping.
