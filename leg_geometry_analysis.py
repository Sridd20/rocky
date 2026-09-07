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

COM_XY      = np.array([0.0, -0.184])   # body-only CoM in XML; full system CoM = -0.120 m

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
    fig, ax = plt.subplots(figsize=(9, 10))
    fig.patch.set_facecolor("#0f1923")
    ax.set_facecolor("#0f1923")

    body_verts = [HIP_POS[l] for l in ["L0", "L1", "L2", "L3", "L4", "L0"]]
    bx = [v[0] * 1000 for v in body_verts]
    by = [v[1] * 1000 for v in body_verts]
    ax.fill(bx[:-1], by[:-1], color="#1e3050", alpha=0.6, zorder=1)
    ax.plot(bx, by, color="#3a7bbf", lw=1.5, zorder=2)

    tri_pts = np.array([FOOT_POS[t] * 1000 for t in TRIPOD_LEGS])
    cx_t, cy_t = tri_pts[:, 0].mean(), tri_pts[:, 1].mean()
    order_t = np.argsort(np.arctan2(tri_pts[:, 1] - cy_t, tri_pts[:, 0] - cx_t))
    tri_sorted = tri_pts[order_t]
    poly_tri = plt.Polygon(tri_sorted, closed=True,
                           facecolor="#1a4a2a", edgecolor="#2ecc71",
                           lw=1.8, alpha=0.5, zorder=3)
    ax.add_patch(poly_tri)

    colors = {"L0": "#e74c3c", "L1": "#e74c3c",
              "L2": "#2ecc71", "L3": "#2ecc71", "L4": "#2ecc71"}

    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        hx, hy = HIP_POS[leg]
        fx, fy = FOOT_POS[leg]
        ax.plot([hx * 1000, fx * 1000], [hy * 1000, fy * 1000],
                color=colors[leg], lw=2.5, zorder=4, alpha=0.85)

    offsets = {"L0": (-20, 8), "L1": (8, 8),
               "L2": (8, -14), "L3": (0, -16), "L4": (-24, -14)}

    for leg in ["L0", "L1", "L2", "L3", "L4"]:
        fx, fy = FOOT_POS[leg]
        hx, hy = HIP_POS[leg]
        fc = "#e74c3c" if leg in SWING_LEGS else "#2ecc71"
        ax.scatter(fx * 1000, fy * 1000, s=220, color=fc, zorder=6,
                   edgecolors="white", lw=1.2)
        ax.scatter(hx * 1000, hy * 1000, s=80, color=fc, zorder=5,
                   edgecolors="#aaaaaa", lw=0.8, alpha=0.7)
        ox, oy = offsets[leg]
        ax.annotate(f"{leg}\n({fx*1000:.0f},{fy*1000:.0f})",
                    xy=(fx * 1000, fy * 1000),
                    xytext=(fx * 1000 + ox, fy * 1000 + oy),
                    color="white", fontsize=7.5, ha="center",
                    fontfamily="monospace",
                    bbox=dict(boxstyle="round,pad=0.25", fc="#1a1a2e",
                              ec=fc, lw=0.9))

    f0 = FOOT_POS["L0"] * 1000
    f1 = FOOT_POS["L1"] * 1000
    d_foot_mm = np.linalg.norm(f1 - f0)
    mid_ft = (f0 + f1) / 2
    ax.annotate("", xy=f1, xytext=f0,
                arrowprops=dict(arrowstyle="<->", color="#f39c12", lw=2.2))
    ax.text(mid_ft[0], mid_ft[1] + 16,
            f"L0-L1 foot span:  {d_foot_mm:.1f} mm",
            color="#f39c12", fontsize=9.5, ha="center", fontweight="bold",
            bbox=dict(fc="#1a1a2e", ec="#f39c12", lw=0.9, pad=3))

    h0a = np.array(HIP_POS["L0"]) * 1000
    h1a = np.array(HIP_POS["L1"]) * 1000
    d_hip_mm = np.linalg.norm(h1a - h0a)
    ax.annotate("", xy=h1a, xytext=h0a,
                arrowprops=dict(arrowstyle="<->", color="#f39c12",
                                lw=1.2, linestyle="dashed"))
    ax.text((h0a[0] + h1a[0]) / 2, (h0a[1] + h1a[1]) / 2 - 10,
            f"Hip span: {d_hip_mm:.1f} mm",
            color="#f39c12", fontsize=7.5, ha="center", alpha=0.85)

    cx_m, cy_m = COM_XY * 1000
    ax.scatter(cx_m, cy_m, s=320, marker="*", color="#f1c40f",
               zorder=10, edgecolors="white", lw=1.0)
    ax.text(cx_m + 11, cy_m + 5, "CoM  (0, -8 mm)",
            color="#f1c40f", fontsize=8.5, fontweight="bold",
            bbox=dict(fc="#1a1a2e", ec="#f1c40f", lw=0.8, pad=2))

    ax.scatter(0, 0, s=60, marker="+", color="white", zorder=8, lw=1.5)

    ax.plot([f0[0], f1[0]], [f0[1], f1[1]], "--",
            color="#e74c3c", lw=1.4, alpha=0.5, zorder=3)

    tc = np.array([FOOT_POS[t] for t in TRIPOD_LEGS]).mean(axis=0) * 1000
    ax.scatter(tc[0], tc[1], s=110, marker="x",
               color="#2ecc71", zorder=7, lw=2.2)
    ax.text(tc[0] + 9, tc[1] - 12, "Tripod centroid",
            color="#2ecc71", fontsize=7.5)

    swing_patch  = mpatches.Patch(color="#e74c3c", label="Swing legs (L0, L1)")
    tripod_patch = mpatches.Patch(color="#2ecc71", label="Tripod legs (L2, L3, L4)")
    com_patch    = mpatches.Patch(color="#f1c40f", label="Centre of Mass")
    span_patch   = mpatches.Patch(color="#f39c12", label="L0-L1 span")
    ax.legend(handles=[swing_patch, tripod_patch, com_patch, span_patch],
              loc="upper right", fontsize=8.5,
              facecolor="#1a1a2e", edgecolor="#3a7bbf", labelcolor="white")

    ax.set_xlabel("X (mm)", color="white", fontsize=10)
    ax.set_ylabel("Y (mm)  --  front of robot toward +Y", color="white", fontsize=10)
    ax.set_title("Pentapod -- Top-Down Leg Geometry\n"
                 "Red = Swing pair (L0,L1)  |  Green = Tripod (L2,L3,L4)  |  Star = CoM",
                 color="white", fontsize=11, pad=12)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_color("#3a7bbf")
    ax.set_aspect("equal")
    ax.grid(True, color="#1e2a3a", lw=0.7, alpha=0.6)

    plt.tight_layout()
    plt.savefig("leg_geometry.png", dpi=160, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print("\nDiagram saved -> leg_geometry.png")
    plt.close()


if __name__ == "__main__":
    print_report()
    draw_diagram()
