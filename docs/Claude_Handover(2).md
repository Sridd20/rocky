# Claude Handover — Sridd's Pentapod Project

**Prepared by:** Claude
**Date:** August 23, 2026
**Purpose:** Context transfer for continued assistance on Sridd's Pentapod robotics project.

---

## Who Sridd Is (Quick Context)

Final-year B.Tech ECE student at MBCET, Thiruvananthapuram (graduating 2027). Completed a software engineering internship at HMA (ERP systems, ML forecasting, IoT). Currently juggling a final-year project (fault detection for DC-DC buck converters using ML on embedded hardware) alongside Pentapod. Comfortable with embedded systems, ML pipelines, and cloud infra (AWS IoT, ESP32, MuJoCo). Prefers terse, direct answers — supplies his own drafts/documents for refinement rather than starting from scratch, and pushes back when guidance feels inefficient. Don't over-explain basics he already knows.

---

## Project: Pentapod — 5-Legged Autonomous Robot

**Concept:** A five-legged robot inspired by Rocky (*Project Hail Mary*), entered in the Arduino Physical AI Challenge (deadline was July 31 — verify current status/deadline with Sridd, as this may have shifted).

### Key Specs
- **Controller:** Arduino UNO Q — dual-brain board (Qualcomm QRB2210 Linux side for vision/audio/AI + STM32U585 MCU side for real-time servo timing, linked via internal RPC "Bridge")
- **Core subsystems:** 5-leg custom gait, A→B waypoint navigation, camera-based obstacle/human detection, fixed-vocabulary voice commands, stretch goal of LIDAR/ToF ranging and stationary gesture-mimicry mode (2 legs mirror upper-body movement while 3 legs hold support stance)
- **Servo count:** 10–15 across 5 legs (2–3 DOF per leg)

### Why UNO Q Specifically
This is the single most important architectural insight for this project: **UNO Q is justified because Pentapod has a genuine dual-workload requirement** — vision/audio ML running continuously on the Linux side *while* real-time servo control runs concurrently on the MCU side. Any future microcontroller recommendation should be checked against this litmus test: does the project need simultaneous Linux-class AI compute + deterministic real-time control, or just one of the two? If Sridd ever scopes down subsystems (e.g., drops voice or vision), this justification should be re-evaluated — a lighter board might suffice.

**Alternatives discussed, ranked:**
- Raspberry Pi 5 + separate MCU (RP2040/STM32) — more raw compute, but Sridd would own the inter-processor communication layer himself (added integration risk)
- Jetson Orin Nano/NX + separate MCU — best for real neural nets, but pricier/heavier, still needs a second MCU
- ESP32-S3 alone — cheap, one-chip, but compute headroom is thin for simultaneous vision+audio+15 servos
- Two Arduino/Teensy boards over serial — cheapest but most DIY integration work, loses most vision capability

**Conclusion:** Stick with UNO Q unless Sridd hits a specific compute wall (e.g., gesture mimicry needing a heavier model than QRB2210 can handle), in which case Pi 5 + RP2040 is the natural upgrade path since it preserves the same dual-brain split.

### Novelty (for competition/report framing)
The individual pieces (camera avoidance, LIDAR, voice commands) are common in student projects. **The actual novelty is the 5-leg count itself** — odd leg counts break standard tripod/trot gaits used everywhere else, making gait design a genuinely open problem with no template to copy (confirmed as "Medium" risk in his own feasibility table). Secondary novelty: the gesture-mimicry/support-leg split (2 legs as "arms," 3 as base) solves the interaction-mode stability problem as a side effect of the gesture feature, with little prior art at student-project level.

**Advice:** Lead the novelty pitch with the gait problem, not the individual sensors — judges will have seen camera+LIDAR+voice bots before, but not a genuinely unsolved 5-leg gait.

### Difficulty/Complexity Assessment
**Difficulty: 8.5/10 | Complexity: 9/10**
- 5-leg gait: open design problem, no standard reference (highest risk item — recommend Sridd builds/demonstrates this first before layering stretch goals, which is also his own stated plan in the proposal)
- Real concurrency: vision + audio + real-time servo timing running simultaneously across two processors — nontrivial cross-processor debugging
- Mechanical risk stacks on top of software risk: 10-15 servos = power budget, wiring, backlash concerns
- Five major subsystems = high integration risk even at reduced scope

### Simulation Recommendation
For finding optimal stationary 3-leg stance (stability) and validating gait code before buying hardware:
- **Support-polygon stability check:** Compute convex hull of the 3 stance-leg foot points, project center of mass down, verify it's inside the polygon, optimize for maximum stability margin (distance from CoM to nearest edge). This is a geometry/optimization problem — do it directly in Python/NumPy/SciPy, not a full physics sim.
- **Full physics validation: MuJoCo** (recommended — Sridd already knows it from a prior project, no new learning curve; built specifically for contact-rich legged locomotion; Python API integrates directly with the stability-margin code above).
- **Workflow suggested:** Build rough MJCF/URDF model with per-part mass (real servo datasheet weights + estimated structural mass) → simulate standing on 3 legs, check CoM/torque requirements against servo specs *before* purchasing servos → script gait sequence, run full walk cycle, check for tipping/servo overload → iterate in code, not hardware.
- **Alternatives if needed:** Webots (easier GUI-based robot building), PyBullet (lighter, more example gait code to adapt), Gazebo+ROS2 (only worth it if final stack runs ROS2 anyway). CAD tools (Fusion 360/SolidWorks) are complementary for getting accurate mass/inertia values to feed into MuJoCo, not a replacement for it.

---

## Advice for Next Assistant

1. **Microcontroller selection should always be argued from workload, not brand/capability.** The recurring theme in discussions so far: match compute tier to what's actually running concurrently. Don't default to the fanciest board — UNO Q is right for Pentapod specifically because of its dual vision+real-time-control need, not because it's the newest/most capable option.

2. **MuJoCo is Sridd's established simulation tool** from prior work — default to MuJoCo-based suggestions for gait/stability simulation unless there's a specific reason to introduce a new tool.

3. **The highest-risk item is the 5-leg gait itself.** Sridd's own proposal already recommends building/validating that first before layering stretch goals (LIDAR, gesture mimicry). Any implementation help should respect this sequencing — don't help him get distracted by stretch-goal features before the core gait is proven stable in simulation.

4. **No hardware has been purchased yet** as of this handover — all work so far has been proposal/planning/architecture-level. The next practical step Sridd was moving toward was pre-purchase simulation (MuJoCo model with realistic per-part mass) to validate servo torque requirements before buying components.

5. **Communication style:** Sridd wants direct, information-dense answers. Skip preamble, use tables/comparisons where they clarify trade-offs, and don't pad responses with reassurance or hedging he hasn't asked for.
