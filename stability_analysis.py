"""
stability_analysis.py
=====================
Reproduces the Liu et al. (2012) Sequential 3+2 Gait stability analysis
for Pentapod, using the robot's actual foot positions from pentapod.xml.

For each of the 120 gait permutations (5! orderings of the 5 swing-pairs),
and for each of the 10 states per gait cycle, computes the stability margin:
  = min distance from CoM projection to any edge of the 3-foot support triangle.

Outputs:
  - stability_results.csv  : full 120×10 matrix
  - top_gaits.txt          : top gaits ranked by min stability margin

Usage:
    python stability_analysis.py
"""

import itertools
import math
import numpy as np
import csv
from pathlib import Path

# ---------------------------------------------------------------------------
# Robot geometry (from pentapod.xml, all in metres)
# Foot positions in neutral stance: radial distance from centre * unit vector
# Legs 0-4 at 0°, 72°, 144°, 216°, 288°
# Foot radial reach = coxa(0.045) + femur_x(0.090) + tibia_x(0.060) = 0.195 m
# (approximate, ignoring the downward tilt of femur/tibia — horizontal projection)
# ---------------------------------------------------------------------------

NUM_LEGS = 5
R_FOOT   = 0.195          # metres, horizontal radius to foot tip
R_HIP    = 0.130           # metres, hip attachment radius

# Foot angles in radians (leg 0 = 0 rad, leg 1 = 72 deg, ...)
LEG_ANGLES_DEG = [i * 72.0 for i in range(NUM_LEGS)]

def foot_position_neutral(leg_idx: int, hip_angle_deg: float = 0.0) -> np.ndarray:
    """XY position of foot tip in neutral stance, no swing offset."""
    angle_rad = math.radians(LEG_ANGLES_DEG[leg_idx] + hip_angle_deg)
    return np.array([R_FOOT * math.cos(angle_rad),
                     R_FOOT * math.sin(angle_rad)])

# Neutral foot positions (indexed 0..4)
NEUTRAL_FEET = np.array([foot_position_neutral(i) for i in range(NUM_LEGS)])

# CoM projection in neutral stance = centre of base = (0, 0)
COM = np.array([0.0, 0.0])

# Pace length per stride (metres) — paper uses 100 mm
PACE_LENGTH = 0.10

# CoM advances 0.4 * PACE_LENGTH per stride phase (from paper Section III)
COM_ADVANCE = 0.4 * PACE_LENGTH

# ---------------------------------------------------------------------------
# Foot-pair definitions (non-adjacent pairs only are valid per paper rule c)
# Pairs numbered 1..5 mapping to leg indices 0..4
# Non-adjacent pairs: pairs where |leg_a - leg_b| != 1 (mod 5)
# ---------------------------------------------------------------------------

def is_adjacent(a: int, b: int) -> bool:
    return abs(a - b) % NUM_LEGS == 1 or abs(a - b) % NUM_LEGS == NUM_LEGS - 1

ALL_PAIRS = [(a, b) for a in range(NUM_LEGS)
                     for b in range(a+1, NUM_LEGS)
                     if not is_adjacent(a, b)]
# Should give exactly 5 non-adjacent pairs for a pentagon
assert len(ALL_PAIRS) == 5, f"Expected 5 non-adj pairs, got {len(ALL_PAIRS)}: {ALL_PAIRS}"

# ---------------------------------------------------------------------------
# Stability margin: min signed distance from point P to edges of triangle ABC
# Positive = inside, negative = outside
# ---------------------------------------------------------------------------

def point_to_segment_dist(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Signed distance from point p to line through a,b (positive = left side)."""
    ab = b - a
    normal = np.array([-ab[1], ab[0]])
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    return float(np.dot(p - a, normal))

def stability_margin(com: np.ndarray, support_feet: np.ndarray) -> float:
    """
    Minimum distance from com to any edge of the convex hull of support_feet.
    Negative if com is outside the polygon.
    support_feet: (3,2) array of XY positions
    """
    n = len(support_feet)
    # Ensure CCW winding
    cx = support_feet[:, 0].mean()
    cy = support_feet[:, 1].mean()
    angles = np.arctan2(support_feet[:, 1] - cy, support_feet[:, 0] - cx)
    order = np.argsort(angles)
    pts = support_feet[order]

    min_dist = math.inf
    for i in range(n):
        a = pts[i]
        b = pts[(i + 1) % n]
        d = point_to_segment_dist(com, a, b)
        min_dist = min(min_dist, d)
    return min_dist

# ---------------------------------------------------------------------------
# Simulate a gait sequence
# gait_seq: list of 5 pair indices (0-based into ALL_PAIRS)
# Returns: list of 10 stability margins (pre- and post-stride for each phase)
#          None entries where com exits polygon
# ---------------------------------------------------------------------------

def simulate_gait(gait_seq: list) -> list:
    feet = NEUTRAL_FEET.copy()  # shape (5,2)
    com  = COM.copy()

    # Walking direction = 90 deg = +Y axis (matching paper's default)
    walk_dir = np.array([0.0, 1.0])

    margins = []

    for phase_idx, pair_idx in enumerate(gait_seq):
        swing_a, swing_b = ALL_PAIRS[pair_idx]
        support_indices = [i for i in range(NUM_LEGS)
                           if i != swing_a and i != swing_b]
        support_feet = feet[support_indices]

        # State 1 (pre-stride): feet in current positions, com hasn't moved yet
        m_pre = stability_margin(com, support_feet)
        margins.append(m_pre)

        # Stride: swing feet move forward by PACE_LENGTH in walk_dir
        feet[swing_a] += walk_dir * PACE_LENGTH
        feet[swing_b] += walk_dir * PACE_LENGTH

        # CoM advances 0.4 * PACE_LENGTH
        com = com + walk_dir * COM_ADVANCE

        # State 2 (post-stride)
        m_post = stability_margin(com, support_feet)
        margins.append(m_post)

    return margins  # length = 10

# ---------------------------------------------------------------------------
# Run all 120 permutations
# ---------------------------------------------------------------------------

def run_all_gaits():
    pair_indices = list(range(len(ALL_PAIRS)))  # [0,1,2,3,4]
    all_perms = list(itertools.permutations(pair_indices))  # 120 permutations

    results = []
    for perm in all_perms:
        margins = simulate_gait(list(perm))
        min_m = min(margins)
        sum_m = sum(margins)
        stable = all(m > 0 for m in margins)
        results.append({
            "gait": perm,
            "margins": margins,
            "min_margin": min_m,
            "sum_margin": sum_m,
            "stable": stable,
        })

    return results

def pair_name(pair_idx: int) -> str:
    a, b = ALL_PAIRS[pair_idx]
    return f"({a+1},{b+1})"  # 1-indexed like the paper

def main():
    print("Running stability analysis for all 120 gaits...")
    results = run_all_gaits()

    # Filter stable gaits
    stable = [r for r in results if r["stable"]]
    print(f"\nStable gaits (all 10 states margin > 0): {len(stable)} / 120")

    # Sort by min_margin descending
    stable.sort(key=lambda r: r["min_margin"], reverse=True)

    # Print top 20
    print(f"\n{'Rank':<5} {'Gait Sequence':<35} {'Min Margin (m)':<18} {'Sum Margin (m)'}")
    print("-" * 75)
    for rank, r in enumerate(stable[:20], 1):
        seq = "-".join(pair_name(p) for p in r["gait"])
        print(f"{rank:<5} {seq:<35} {r['min_margin']*1000:>10.2f} mm      "
              f"{r['sum_margin']*1000:.1f} mm")

    # Save CSV
    csv_path = Path("stability_results.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["gait_seq", "stable", "min_margin_mm", "sum_margin_mm"] + \
                 [f"state_{i+1}" for i in range(10)]
        writer.writerow(header)
        for r in results:
            seq = "-".join(pair_name(p) for p in r["gait"])
            row = [seq, r["stable"],
                   round(r["min_margin"]*1000, 3),
                   round(r["sum_margin"]*1000, 3)]
            row += [round(m*1000, 3) for m in r["margins"]]
            writer.writerow(row)
    print(f"\nFull results saved to: {csv_path.resolve()}")

    # Save best gait info
    best = stable[0] if stable else None
    if best:
        txt_path = Path("top_gaits.txt")
        with open(txt_path, "w") as f:
            f.write("TOP STABLE GAITS — Pentapod 3+2 Gait Analysis\n")
            f.write("=" * 60 + "\n\n")
            f.write(f"Non-adjacent swing pairs: {[f'{pair_name(i)}' for i in range(5)]}\n\n")
            for rank, r in enumerate(stable[:10], 1):
                seq = "-".join(pair_name(p) for p in r["gait"])
                f.write(f"Rank {rank}: {seq}\n")
                f.write(f"  Min margin: {r['min_margin']*1000:.2f} mm\n")
                f.write(f"  Sum margin: {r['sum_margin']*1000:.1f} mm\n")
                f.write(f"  State margins: {[round(m*1000,1) for m in r['margins']]}\n\n")
        print(f"Top gaits saved to: {txt_path.resolve()}")

    print(f"\nBEST GAIT: {'-'.join(pair_name(p) for p in best['gait'])}")
    print(f"Min stability margin: {best['min_margin']*1000:.2f} mm")

if __name__ == "__main__":
    main()
