"""
view_pentapod.py — Launch Pentapod in MuJoCo interactive viewer.

Usage:
    python view_pentapod.py              # free-fall mode (freejoint enabled)
    python view_pentapod.py --static     # hold base fixed, inspect stance

Controls in viewer: mouse drag to orbit, scroll to zoom, space to pause.
"""

import sys
import argparse
import math
import mujoco
import mujoco.viewer

XML_PATH = "pentapod.xml"

# ---------------------------------------------------------------------------
# Stance angles (degrees) — robot standing on all 5 legs.
# knee negative = leg pushed downward.  ankle compensates.
# Tweak these until feet are flat on the floor in the viewer.
# ---------------------------------------------------------------------------
STANCE = {
    "knee":  -40.0,   # degrees: femur pitches down
    "ankle":  35.0,   # degrees: tibia straightens toward ground
    "hip":     0.0,   # degrees: legs point straight out radially
}


def set_stance(data: mujoco.MjData, model: mujoco.MjModel) -> None:
    """Set all actuator targets to the default standing pose.
    Note: MuJoCo position actuators for hinge joints expect RADIANS."""
    for i in range(5):
        data.ctrl[model.actuator(f"act_hip_{i}").id]   = math.radians(STANCE["hip"])
        data.ctrl[model.actuator(f"act_knee_{i}").id]  = math.radians(STANCE["knee"])
        data.ctrl[model.actuator(f"act_ankle_{i}").id] = math.radians(STANCE["ankle"])


def print_com_and_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Print centre-of-mass and which feet are in contact."""
    com = data.subtree_com[model.body("base").id]
    print(f"CoM: x={com[0]:.3f}  y={com[1]:.3f}  z={com[2]:.3f}")

    in_contact = []
    for i in range(5):
        touch_id = model.sensor(f"touch_foot_{i}").id
        force    = data.sensordata[touch_id]
        if force > 0.01:
            in_contact.append(i)
    print(f"Feet in contact: {in_contact}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--static", action="store_true",
                        help="Disable freejoint and fix the base in place")
    args = parser.parse_args()

    # Load model
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data  = mujoco.MjData(model)

    # Optional: fix base for static stance inspection
    if args.static:
        root_jnt = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
        if root_jnt >= 0:
            # Zero out all freejoint DoF so body stays put
            data.qpos[:7] = [0, 0, 0.25, 1, 0, 0, 0]   # x y z qw qx qy qz
        print("[STATIC MODE] Base fixed at z=0.25 m")

    # Apply initial stance pose
    set_stance(data, model)
    mujoco.mj_forward(model, data)

    step_count = 0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Pentapod viewer running. Close window to exit.")
        print_com_and_contacts(model, data)

        # Pause for 1.5 real seconds at spawn so you can see the robot
        # in the air before it drops — viewer renders but sim doesn't step.
        import time
        t0 = time.time()
        while viewer.is_running() and (time.time() - t0) < 1.5:
            viewer.sync()

        print("--- dropping ---")

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

            step_count += 1
            # Print diagnostics every 2 seconds of sim time
            if step_count % 1000 == 0:
                print_com_and_contacts(model, data)


if __name__ == "__main__":
    main()
