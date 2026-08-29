"""
gait_runner.py — Pentapod Sequential 3+2 Gait
Two modes: stationary (tripod base + 2 arm legs) and walk.

Walking uses correct lift geometry and a push-step mechanism:
  Support legs: hold planted hip angle → body pushed forward by reaction force
  Swing legs:   lift (knee +ve = foot UP) then step to foot+step_vec target
"""

import argparse
import math
import numpy as np
import mujoco
import mujoco.viewer

XML_PATH = "pentapod.xml"

NUM_LEGS       = 5
LEG_ANGLES_DEG = [i * 72.0 for i in range(NUM_LEGS)]   # 0,72,144,216,288
ALL_PAIRS      = [(0,2),(1,3),(2,4),(3,0),(4,1)]        # non-adjacent swing pairs

# Robot geometry (metres, from pentapod.xml)
R_HIP        = 0.130   # hip attachment radius
R_COXA       = 0.045   # coxa link length
FEMUR_REACH  = 0.150   # horizontal reach of femur+tibia at nominal stance angles
R_FOOT       = R_HIP + R_COXA + FEMUR_REACH  # total foot radius = 0.325 m

# ── Stance joint angles ──────────────────────────────────────────────────────
STANCE_HIP   =  0.0
STANCE_KNEE  = -40.0   # knee=-40 → femur angled 40° down → foot on ground
STANCE_ANKLE =  35.0   # ankle=35 → foot pitched up 35° → flat contact

# ── Lift joint angles ───────────────────────────────────────────────────────
# Knee axis "0 1 0".  MORE NEGATIVE knee → femur tilts UP → foot rises.
#   Verified: knee=-65° raises foot 4.65cm above ground level.
#   knee=+15° (previous bug) drove foot 7.3cm UNDERGROUND — never use positive.
# Ankle: lower value = foot pulls up toward tibia = extra clearance.
LIFT_KNEE    = -55.0   # knee more negative = femur tilts up = foot rises ~3cm clear
LIFT_ANKLE   =  10.0   # reduced from 35° → foot tucks up for extra clearance

# ── Gait parameters ──────────────────────────────────────────────────────────
STEP_LENGTH     = 0.04   # metres per stride (smaller = more stable)
STEPS_PER_PHASE = 500    # sim steps per gait phase
SETTLE_STEPS    = 1200   # steps to let robot settle before gait starts

# ── Stationary mode ──────────────────────────────────────────────────────────
TRIPOD_LEGS = [1, 3, 4]   # 72°, 216°, 288° — wide rear spread
ARM_LEGS    = [0, 2]       # 0°, 144° — forward-facing non-adjacent pair
ARM_KNEE    = -70.0
ARM_ANKLE   =  -5.0


# ────────────────────────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────────────────────────

def set_ctrl(data, model, name: str, val_deg: float):
    """Set actuator ctrl. val_deg is in DEGREES; converts to radians for MuJoCo."""
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid >= 0:
        data.ctrl[aid] = math.radians(val_deg)

def get_joint_angle_deg(data, model, joint_name: str) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    addr = model.jnt_qposadr[jid]
    return math.degrees(data.qpos[addr])

def get_body_pose(data):
    """(x, y, yaw) of base body from freejoint qpos."""
    x, y = data.qpos[0], data.qpos[1]
    qw,qx,qy,qz = data.qpos[3],data.qpos[4],data.qpos[5],data.qpos[6]
    yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
    return x, y, yaw

def hip_origin_world(leg: int, bx, by, byaw):
    ang = math.radians(LEG_ANGLES_DEG[leg]) + byaw
    return bx + R_HIP*math.cos(ang), by + R_HIP*math.sin(ang)

def hip_angle_for_foot(foot_xy, leg: int, bx, by, byaw) -> float:
    """Hip joint angle (deg) to aim leg toward foot_xy in world frame."""
    hx, hy = hip_origin_world(leg, bx, by, byaw)
    target_ang = math.atan2(foot_xy[1]-hy, foot_xy[0]-hx)
    ref_ang    = math.radians(LEG_ANGLES_DEG[leg]) + byaw
    delta = (target_ang - ref_ang + math.pi) % (2*math.pi) - math.pi
    return max(-45.0, min(45.0, math.degrees(delta)))

def neutral_foot_world(leg: int, bx, by, byaw) -> np.ndarray:
    ang = math.radians(LEG_ANGLES_DEG[leg]) + byaw
    return np.array([bx + R_FOOT*math.cos(ang), by + R_FOOT*math.sin(ang)])

def all_stance(data, model):
    for i in range(NUM_LEGS):
        set_ctrl(data, model, f"act_hip_{i}",   STANCE_HIP)
        set_ctrl(data, model, f"act_knee_{i}",  STANCE_KNEE)
        set_ctrl(data, model, f"act_ankle_{i}", STANCE_ANKLE)

def lerp(a, b, t): return a + (b-a)*t

def step_sim(data, model, viewer, n=1):
    for _ in range(n):
        if not viewer.is_running(): return False
        mujoco.mj_step(model, data)
        viewer.sync()
    return True

def select_forward_pair(walk_dir_deg: float) -> int:
    wr = math.radians(walk_dir_deg)
    return max(range(5), key=lambda i:
        math.cos(math.radians(LEG_ANGLES_DEG[ALL_PAIRS[i][0]])-wr) +
        math.cos(math.radians(LEG_ANGLES_DEG[ALL_PAIRS[i][1]])-wr))

def build_gait_sequence(walk_dir_deg: float) -> list:
    first = select_forward_pair(walk_dir_deg)
    rest  = sorted([i for i in range(5) if i != first],
                   key=lambda i: -(
                       math.cos(math.radians(LEG_ANGLES_DEG[ALL_PAIRS[i][0]])-math.radians(walk_dir_deg))+
                       math.cos(math.radians(LEG_ANGLES_DEG[ALL_PAIRS[i][1]])-math.radians(walk_dir_deg))))
    return [first]+rest


# ────────────────────────────────────────────────────────────────────────────
# STATIONARY MODE
# ────────────────────────────────────────────────────────────────────────────

def stationary_mode(model, data, viewer):
    print(f"\n[STATIONARY MODE]")
    print(f"  Tripod: legs {TRIPOD_LEGS} ({[int(LEG_ANGLES_DEG[i]) for i in TRIPOD_LEGS]}°)")
    print(f"  Arms:   legs {ARM_LEGS}    ({[int(LEG_ANGLES_DEG[i]) for i in ARM_LEGS]}°)")

    pts = np.array([[R_FOOT*math.cos(math.radians(LEG_ANGLES_DEG[i])),
                     R_FOOT*math.sin(math.radians(LEG_ANGLES_DEG[i]))] for i in TRIPOD_LEGS])
    cx,cy = pts[:,0].mean(), pts[:,1].mean()
    pts   = pts[np.argsort(np.arctan2(pts[:,1]-cy, pts[:,0]-cx))]
    min_m = min(float(np.dot(np.array([0,0])-pts[i],
                             np.array([-(pts[(i+1)%3]-pts[i])[1],
                                        (pts[(i+1)%3]-pts[i])[0]]) /
                             (np.linalg.norm(pts[(i+1)%3]-pts[i])+1e-12)))
                for i in range(3))
    print(f"  Stability margin: {min_m*1000:.1f} mm\n")

    all_stance(data, model)
    if not step_sim(data, model, viewer, SETTLE_STEPS): return

    print("Raising arm legs...")
    for s in range(400):
        if not viewer.is_running(): return
        t = math.sin(s/400*math.pi/2)
        for leg in ARM_LEGS:
            set_ctrl(data, model, f"act_knee_{leg}", lerp(STANCE_KNEE, ARM_KNEE, t))
            set_ctrl(data, model, f"act_ankle_{leg}", lerp(STANCE_ANKLE, ARM_ANKLE, t))
        mujoco.mj_step(model, data)
        viewer.sync()

    print("Stationary mode active. Close window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()


# ────────────────────────────────────────────────────────────────────────────
# WALKING MODE
# ────────────────────────────────────────────────────────────────────────────

def walking_mode(model, data, viewer, walk_dir_deg: float, cycles: int, speed: float = 1.0):
    steps_per_phase = max(50, int(STEPS_PER_PHASE / speed))
    gait_seq = build_gait_sequence(walk_dir_deg)

    print(f"\n[WALKING MODE]  direction={walk_dir_deg}  speed={speed:.2f}x  ({steps_per_phase} steps/phase)")
    print(f"Swing pairs: {[ALL_PAIRS[p] for p in gait_seq]}")
    print(f"Forward pair: legs {ALL_PAIRS[gait_seq[0]]} face walk direction\n")

    all_stance(data, model)
    print("Settling...")
    if not step_sim(data, model, viewer, SETTLE_STEPS): return

    print("Walking...\n")

    R_LEG = R_COXA + FEMUR_REACH   # horizontal reach hip->foot = 0.195 m

    def swing_hip_deg(leg_idx):
        """Hip angle (deg) to step leg forward by STEP_LENGTH in walk_dir_deg."""
        phi   = math.radians(LEG_ANGLES_DEG[leg_idx])
        theta = math.radians(walk_dir_deg)
        vx = R_LEG * math.cos(phi) + STEP_LENGTH * math.cos(theta)
        vy = R_LEG * math.sin(phi) + STEP_LENGTH * math.sin(theta)
        hip_rad = math.atan2(vy, vx) - phi
        hip_rad = (hip_rad + math.pi) % (2*math.pi) - math.pi
        return max(-35.0, min(35.0, math.degrees(hip_rad)))

    for cycle in range(cycles):
        for phase_num, pair_idx in enumerate(gait_seq):
            if not viewer.is_running(): return

            swing_a, swing_b = ALL_PAIRS[pair_idx]
            support = [i for i in range(NUM_LEGS) if i not in (swing_a, swing_b)]

            tgt_a = swing_hip_deg(swing_a)
            tgt_b = swing_hip_deg(swing_b)

            print(f"  Cycle {cycle+1} Phase {phase_num+1}/5 | "
                  f"Swing {swing_a}({tgt_a:+.1f}deg) {swing_b}({tgt_b:+.1f}deg)")

            for step in range(steps_per_phase):
                if not viewer.is_running(): return

                t    = step / steps_per_phase
                lift = math.sin(t * math.pi)

                for leg, tgt_h in [(swing_a, tgt_a), (swing_b, tgt_b)]:
                    set_ctrl(data, model, f"act_hip_{leg}",
                             lerp(STANCE_HIP, tgt_h, t))
                    set_ctrl(data, model, f"act_knee_{leg}",
                             lerp(STANCE_KNEE, LIFT_KNEE, lift))
                    set_ctrl(data, model, f"act_ankle_{leg}",
                             lerp(STANCE_ANKLE, LIFT_ANKLE, lift))

                for leg in support:
                    set_ctrl(data, model, f"act_hip_{leg}",   STANCE_HIP)
                    set_ctrl(data, model, f"act_knee_{leg}",  STANCE_KNEE)
                    set_ctrl(data, model, f"act_ankle_{leg}", STANCE_ANKLE)

                mujoco.mj_step(model, data)
                viewer.sync()

            # Return all legs to neutral then brief settle
            all_stance(data, model)
            if not step_sim(data, model, viewer, 150): return

    print("\nGait complete. Close window to exit.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()


# ────────────────────────────────────────────────────────────────────────────
# MAIN
# ────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode",      choices=["stationary","walk"], default="walk")
    parser.add_argument("--direction", type=float, default=0.0)
    parser.add_argument("--speed",     type=float, default=1.0,
                        help="Speed multiplier: 0.1=very slow, 1.0=normal, 2.0=fast")
    parser.add_argument("--cycles",   type=int,   default=9999)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data  = mujoco.MjData(model)

    # Explicitly set freejoint to z=0.25 m so the robot starts at
    # the correct standing height (don't rely on MuJoCo's qpos0 default).
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    if root_id >= 0:
        addr = model.jnt_qposadr[root_id]
        data.qpos[addr:addr+7] = [0.0, 0.0, 0.25, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        if args.mode == "stationary":
            stationary_mode(model, data, viewer)
        else:
            walking_mode(model, data, viewer, args.direction, args.cycles, args.speed)

if __name__ == "__main__":
    main()
