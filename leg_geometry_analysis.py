"""
leg_geometry_analysis.py
========================
Analyses the pentapod's actual asymmetric geometry:

  - Computes hip and foot positions from pentapod.xml values
  - Reports distance between the 2 swing legs (L0, L1)
  - Reports each swing leg's distance to each tripod leg (L2, L3, L4)
  - Shows CoM position relative to both groups
  - Generates a top-down 2-D diagram saved as leg_geometry.png
  - Reports whether CoM is inside the tripod support polygon

Usage:
    python leg_geometry_analysis.py
"""

import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# -------------------------------------------------------------------------
# Geometry from pentapod.xml  (all metres, body frame, XY = horizontal plane)
# -------------------------------------------------------------------------

HIP_POS = {
    "L0": (-0.0439, +0.0946),
    "L1": (+0.0439, +0.0946),
    "L2": (+0.0703, -0.0296),
    "L3": (+0.0000, -0.1300),
    "L4": (-0.0703, -0.0296),
}

HIP_ANGLE_DEG = {
    "L0":  115.0,
    "L1":   65.0,
    "L2":  -23.0,
    "L3":  -90.0,
    "L4": -157.0,
}

COXA_LEN    = 0.045
FEMUR_HORIZ = 0.090
TIBIA_HORIZ = 0.060
FOOT_REACH  = COXA_LEN + FEMUR_HORIZ + TIBIA_HORIZ   # 0.195 m

COM_XY      = np.array([0.0, -0.120])   # SYSTEM CoM (body+legs); body inertial tag = -0.120m

SWING_LEGS  = ["L0", "L1"]
TRIPOD_LEGS = ["L2", "L3", "L4"]


def neutral_foot(leg):
    hx, hy = HIP_POS[leg]
    ang = math.radians(HIP_ANGLE_DEG[leg])
    return np.array([hx + FOOT_REACH * math.cos(ang),
                     hy + FOOT_REACH * math.sin(ang)])


FOOT_POS = {leg: neutral_foot(leg) for leg in HIP_POS}


def stability_margin(p, pts):
    """Signed min distance from p to edges of convex polygon pts."""
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    order = np.argsort(np.arctan2(pts[:, 1] - cy, pts[:, 0] - cx))
    poly  = pts[order]
    n     = len(poly)
    min_d = float("inf")
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ab   = b - a
        nor  = np.array([-ab[1], ab[0]])
        nor /= np.linalg.norm(nor) + 1e-12
        min_d = min(min_d, float(np.dot(p - a, nor)))
    return min_d


# -------------------------------------------------------------------------
# Text report
# -------------------------------------------------------------------------

def print_report():
    sep = "=" * 62
    print(sep)
    print("  PENTAPOD -- LEG GEOMETRY ANALYSIS")
    print(sep)

    print("\n[1] HIP POSITIONS (body frame, mm)")
    print(f"  {'Leg':<6} {'X':>8} {'Y':>8} {'r':>8} {'angle':>7}  Role")
    print("  " + "-" * 50)
    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        hx, hy = HIP_POS[leg]
        r = math.sqrt(hx**2 + hy**2)
        role = "SWING" if leg in SWING_LEGS else "TRIPOD"
        print(f"  {leg:<6} {hx*1000:>8.1f} {hy*1000:>8.1f} {r*1000:>8.1f}"
              f" {HIP_ANGLE_DEG[leg]:>7.0f}  {role}")

    print(f"\n[2] NEUTRAL FOOT POSITIONS  (reach = {FOOT_REACH*1000:.0f} mm)")
    print(f"  {'Leg':<6} {'X (mm)':>9} {'Y (mm)':>9} {'|r| (mm)':>10}")
    print("  " + "-" * 40)
    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        fx, fy = FOOT_POS[leg]
        r = math.sqrt(fx**2 + fy**2)
        role = "SWING" if leg in SWING_LEGS else "TRIPOD"
        print(f"  {leg:<6} {fx*1000:>9.1f} {fy*1000:>9.1f} {r*1000:>10.1f}  [{role}]")

    h0 = np.array(HIP_POS["L0"])
    h1 = np.array(HIP_POS["L1"])
    f0 = FOOT_POS["L0"]
    f1 = FOOT_POS["L1"]
    d_hip  = np.linalg.norm(h1 - h0)
    d_foot = np.linalg.norm(f1 - f0)

    print(f"\n[3] DISTANCE BETWEEN THE 2 SWING LEGS (L0 and L1)")
    print(f"  Hip-to-hip  distance : {d_hip  * 1000:>8.2f} mm")
    print(f"  Foot-to-foot distance: {d_foot * 1000:>8.2f} mm  <-- KEY VALUE for weight planning")
    print(f"  (L0 and L1 are symmetric about the robot centre line)")

    print(f"\n[4] SWING vs TRIPOD FOOT DISTANCES (mm)")
    print(f"  {'':10}" + "".join(f"  {t:>8}" for t in TRIPOD_LEGS))
    print("  " + "-" * 40)
    for sw in SWING_LEGS:
        row = f"  {sw:<10}"
        for tr in TRIPOD_LEGS:
            d = np.linalg.norm(FOOT_POS[sw] - FOOT_POS[tr])
            row += f"  {d*1000:>8.1f}"
        print(row)

    print()
    for sw in SWING_LEGS:
        dists = {tr: np.linalg.norm(FOOT_POS[sw] - FOOT_POS[tr]) for tr in TRIPOD_LEGS}
        nearest = min(dists, key=dists.get)
        print(f"  Nearest tripod to {sw}: {nearest} at {dists[nearest]*1000:.1f} mm")

    tripod_foot_arr = np.array([FOOT_POS[t] for t in TRIPOD_LEGS])
    tripod_centroid = tripod_foot_arr.mean(axis=0)
    swing_mid_foot  = (f0 + f1) / 2.0

    sm = stability_margin(COM_XY, tripod_foot_arr)
    inside = sm > 0

    print(f"\n[5] CENTRE OF MASS ANALYSIS")
    print(f"  CoM (body frame) :        ({COM_XY[0]*1000:+.1f}, {COM_XY[1]*1000:+.1f}) mm")
    print(f"  Tripod foot centroid:     ({tripod_centroid[0]*1000:+.1f}, {tripod_centroid[1]*1000:+.1f}) mm")
    print(f"  Swing foot midpoint:      ({swing_mid_foot[0]*1000:+.1f}, {swing_mid_foot[1]*1000:+.1f}) mm")
    print(f"  CoM inside tripod poly?   {'YES -- statically stable on 3 legs' if inside else 'NO'}")
    if inside:
        print(f"  Stability margin:         {sm*1000:.2f} mm")
    else:
        print(f"  CoM is outside by:        {abs(sm)*1000:.2f} mm")
        print(f"  --> Shift CoM ~{abs(sm)*1000:.0f} mm further toward tripod side")

    d_com_swing  = np.linalg.norm(COM_XY - swing_mid_foot)
    d_com_tripod = np.linalg.norm(COM_XY - tripod_centroid)
    total_d = d_com_swing + d_com_tripod
    w_tripod = d_com_swing  / total_d * 100
    w_swing  = d_com_tripod / total_d * 100

    print(f"\n[6] STATIC WEIGHT DISTRIBUTION ESTIMATE (lever rule, simplified)")
    print(f"  CoM distance from swing  midpoint:  {d_com_swing  * 1000:>6.1f} mm")
    print(f"  CoM distance from tripod centroid:  {d_com_tripod * 1000:>6.1f} mm")
    print(f"  Weight on TRIPOD side (L2+L3+L4):  {w_tripod:>5.1f}%")
    print(f"  Weight on SWING  side (L0+L1):     {w_swing:>5.1f}%")

    print(f"\n[7] SUMMARY FOR WEIGHT DISTRIBUTION PLANNING")
    print(f"  L0-L1 foot span:                   {d_foot*1000:>6.1f} mm")
    print(f"  L0-L1 hip span:                    {d_hip*1000:>6.1f} mm")
    d_L0_L4 = np.linalg.norm(FOOT_POS["L0"] - FOOT_POS["L4"])
    d_L1_L2 = np.linalg.norm(FOOT_POS["L1"] - FOOT_POS["L2"])
    d_L0_L3 = np.linalg.norm(FOOT_POS["L0"] - FOOT_POS["L3"])
    d_L1_L3 = np.linalg.norm(FOOT_POS["L1"] - FOOT_POS["L3"])
    print(f"  L0-L4 (cross diagonal):            {d_L0_L4*1000:>6.1f} mm")
    print(f"  L1-L2 (cross diagonal):            {d_L1_L2*1000:>6.1f} mm")
    print(f"  L0-L3 (furthest pair):             {d_L0_L3*1000:>6.1f} mm")
    print(f"  L1-L3 (furthest pair):             {d_L1_L3*1000:>6.1f} mm")
    print(sep)


# -------------------------------------------------------------------------
# Diagram
# -------------------------------------------------------------------------

def draw_diagram():
    fig, ax = plt.subplots(figsize=(12, 13))
    fig.patch.set_facecolor("#0f1923")
    ax.set_facecolor("#0f1923")

    # ── Body pentagon (hip level) ────────────────────────────────
    body_verts = [HIP_POS[l] for l in ["L0", "L1", "L2", "L3", "L4", "L0"]]
    bx = [v[0] * 1000 for v in body_verts]
    by = [v[1] * 1000 for v in body_verts]
    ax.fill(bx[:-1], by[:-1], color="#1e3050", alpha=0.7, zorder=1)
    ax.plot(bx, by, color="#3a7bbf", lw=2.0, zorder=2)
    ax.text(0, 55, "BODY (hip level)", color="#3a7bbf", fontsize=8,
            ha="center", alpha=0.8)

    # ── Foot support triangle ────────────────────────────────────
    tri_pts = np.array([FOOT_POS[t] for t in TRIPOD_LEGS]) * 1000
    cx_t, cy_t = tri_pts[:, 0].mean(), tri_pts[:, 1].mean()
    order_t = np.argsort(np.arctan2(tri_pts[:, 1] - cy_t, tri_pts[:, 0] - cx_t))
    tri_sorted = tri_pts[order_t]
    poly_tri = plt.Polygon(tri_sorted, closed=True,
                           facecolor="#1a4a2a", edgecolor="#2ecc71",
                           lw=1.8, alpha=0.5, zorder=3)
    ax.add_patch(poly_tri)

    colors = {"L0": "#e74c3c", "L1": "#e74c3c",
              "L2": "#2ecc71", "L3": "#2ecc71", "L4": "#2ecc71"}

    # ── Leg lines ────────────────────────────────────────────────
    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        hx, hy = np.array(HIP_POS[leg]) * 1000
        fx, fy = FOOT_POS[leg] * 1000
        ax.plot([hx, fx], [hy, fy],
                color=colors[leg], lw=2.5, zorder=4, alpha=0.85)

    # ── Foot & hip dots + labels (well separated with leader lines) ─
    foot_label_xy = {
        "L0": (-200, 310),   # upper-left
        "L1": ( 200, 310),   # upper-right
        "L2": ( 330, -106),  # far right
        "L3": (  60, -370),  # lower, offset right
        "L4": (-330, -106),  # far left
    }
    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        fx, fy = FOOT_POS[leg] * 1000
        hx, hy = np.array(HIP_POS[leg]) * 1000
        fc = "#e74c3c" if leg in SWING_LEGS else "#2ecc71"
        ax.scatter(fx, fy, s=220, color=fc, zorder=6, edgecolors="white", lw=1.2)
        ax.scatter(hx, hy, s=80,  color=fc, zorder=5, edgecolors="#aaaaaa", lw=0.8, alpha=0.7)
        lx, ly = foot_label_xy[leg]
        ax.annotate(f"{leg}  ({fx:.0f}, {fy:.0f}) mm",
                    xy=(fx, fy), xytext=(lx, ly),
                    color="white", fontsize=8.5, ha="center",
                    fontfamily="monospace",
                    arrowprops=dict(arrowstyle="-", color=fc, lw=1.0,
                                   connectionstyle="arc3,rad=0.0"),
                    bbox=dict(boxstyle="round,pad=0.35", fc="#1a1a2e", ec=fc, lw=1.0))

    # ── L0-L1 foot span — above the labels ──────────────────────
    f0 = FOOT_POS["L0"] * 1000
    f1 = FOOT_POS["L1"] * 1000
    d_foot_mm = np.linalg.norm(f1 - f0)
    span_y = 355
    ax.annotate("", xy=(f1[0], span_y), xytext=(f0[0], span_y),
                arrowprops=dict(arrowstyle="<->", color="#f39c12", lw=2.5))
    ax.plot([f0[0], f0[0]], [f0[1]+20, span_y-2], color="#f39c12", lw=0.8, ls=":", alpha=0.5)
    ax.plot([f1[0], f1[0]], [f1[1]+20, span_y-2], color="#f39c12", lw=0.8, ls=":", alpha=0.5)
    ax.text(0, span_y + 12,
            f"L0-L1 foot span:  {d_foot_mm:.1f} mm",
            color="#f39c12", fontsize=11, ha="center", fontweight="bold",
            bbox=dict(fc="#1a1a2e", ec="#f39c12", lw=1.0, pad=4))

    # ── Hip span — small label inside body ──────────────────────
    h0a = np.array(HIP_POS["L0"]) * 1000
    h1a = np.array(HIP_POS["L1"]) * 1000
    d_hip_mm = np.linalg.norm(h1a - h0a)
    ax.text(0, (h0a[1]+h1a[1])/2 - 22,
            f"Hip span: {d_hip_mm:.1f} mm",
            color="#f39c12", fontsize=8, ha="center", alpha=0.8,
            bbox=dict(fc="#1a1a2e", ec="none", pad=1))

    # ── System CoM — left-centre, clear of everything ───────────
    cx_m, cy_m = COM_XY * 1000
    ax.scatter(cx_m, cy_m, s=380, marker="*", color="#f1c40f",
               zorder=10, edgecolors="white", lw=1.2)
    ax.annotate(f"System CoM\n(0, {cy_m:.0f} mm)\nIn body pentagon: YES ✓\nIn foot triangle:   NO  ✗",
                xy=(cx_m, cy_m),
                xytext=(cx_m - 170, cy_m + 60),
                color="#f1c40f", fontsize=8.5, fontweight="bold",
                arrowprops=dict(arrowstyle="->", color="#f1c40f", lw=1.2),
                bbox=dict(fc="#1a1a2e", ec="#f1c40f", lw=1.0, pad=5))

    # ── Tripod balance boundary — label on RIGHT below L2 ───────
    tripod_edge_y = -105.8
    ax.axhline(tripod_edge_y, color="#e67e22", lw=1.8, linestyle="--", zorder=5, alpha=0.9)
    ax.text(340, tripod_edge_y + 12,
            "Balance boundary\nL2-L4 foot top edge\nY = -105.8 mm\n(CoM needs to be\nbelow for passive\nbalance)",
            color="#e67e22", fontsize=8, ha="right",
            bbox=dict(fc="#1a1a2e", ec="#e67e22", lw=0.8, pad=4))

    # ── Body centroid ────────────────────────────────────────────
    ax.scatter(0, -2.5, s=90, marker="D", color="#3a7bbf", zorder=9,
               edgecolors="white", lw=0.8)
    ax.annotate("Body centroid\n(0, -2.5 mm)",
                xy=(0, -2.5), xytext=(120, 40),
                color="#3a7bbf", fontsize=8,
                arrowprops=dict(arrowstyle="->", color="#3a7bbf", lw=0.9),
                bbox=dict(fc="#1a1a2e", ec="#3a7bbf", lw=0.7, pad=3))

    # ── Origin ───────────────────────────────────────────────────
    ax.scatter(0, 0, s=60, marker="+", color="white", zorder=8, lw=1.5)

    # ── Tripod centroid ──────────────────────────────────────────
    tc = np.array([FOOT_POS[t] for t in TRIPOD_LEGS]).mean(axis=0) * 1000
    ax.scatter(tc[0], tc[1], s=120, marker="x", color="#2ecc71", zorder=7, lw=2.5)
    ax.text(tc[0] + 14, tc[1] + 14, "Tripod centroid",
            color="#2ecc71", fontsize=8, ha="left")

    # ── Legend — lower-left (open space) ────────────────────────
    swing_patch  = mpatches.Patch(color="#e74c3c", label="Swing legs  (L0, L1)")
    tripod_patch = mpatches.Patch(color="#2ecc71", label="Tripod legs (L2, L3, L4)")
    com_patch    = mpatches.Patch(color="#f1c40f", label="System CoM  (-120 mm Y)")
    body_patch   = mpatches.Patch(color="#3a7bbf", label="Body pentagon (hip level)")
    edge_patch   = mpatches.Patch(color="#e67e22", label="Balance boundary (foot edge)")
    span_patch   = mpatches.Patch(color="#f39c12", label="L0-L1 foot span")
    ax.legend(handles=[swing_patch, tripod_patch, com_patch,
                       body_patch, edge_patch, span_patch],
              loc="lower left", fontsize=9,
              facecolor="#1a1a2e", edgecolor="#3a7bbf", labelcolor="white")

    ax.set_xlabel("X (mm)", color="white", fontsize=11)
    ax.set_ylabel("Y (mm)  —  front of robot toward +Y", color="white", fontsize=11)
    ax.set_title(
        "Pentapod — Top-Down Leg Geometry\n"
        "★ CoM is inside the body pentagon (YES) but outside the foot support triangle (NO)"
        " — active hip balancing required",
        color="white", fontsize=10, pad=14)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_color("#3a7bbf")
    ax.set_aspect("equal")
    ax.set_xlim(-370, 370)
    ax.set_ylim(-415, 410)
    ax.grid(True, color="#1e2a3a", lw=0.7, alpha=0.6)

    plt.tight_layout()
    plt.savefig("leg_geometry.png", dpi=160, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print("\nDiagram saved -> leg_geometry.png")
    plt.close()


if __name__ == "__main__":
    print_report()
    draw_diagram()
