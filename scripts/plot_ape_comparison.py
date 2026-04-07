"""
APE Comparison Plot: dynamic_v3 Baseline vs Scan-Filter

Dynamic-event timestamps are given in rosbag simulation time.
The bag's first pose is recorded at sim_time = BAG_T0 seconds,
so all event intervals are shifted by -BAG_T0 to align with
the evo `seconds_from_start` axis (which starts at 0).

Usage:
    python3 scripts/plot_ape_comparison.py
Input:
    eval/plots/dynamic_v3_baseline/ape.zip
    eval/plots/dynamic_v3_filtered_v3/ape.zip
Output (both locations):
    eval/plots/ape_comparison_dynamic_v3.{png,pdf}   (local, gitignored)
    docs/assets/ape_comparison_dynamic_v3.png         (tracked, for README)
"""

import zipfile, json, pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import MultipleLocator

# ── paths ─────────────────────────────────────────────────────────────────────
REPO_ROOT    = pathlib.Path(__file__).parent.parent
BASELINE_ZIP = REPO_ROOT / "eval/plots/dynamic_v3_baseline/ape.zip"
FILTERED_ZIP = REPO_ROOT / "eval/plots/dynamic_v3_filtered_v3/ape.zip"
OUT_DIR      = REPO_ROOT / "eval/plots"
DOCS_DIR     = REPO_ROOT / "docs/assets"
OUT_DIR.mkdir(parents=True, exist_ok=True)
DOCS_DIR.mkdir(parents=True, exist_ok=True)

# ── load ──────────────────────────────────────────────────────────────────────
def load_zip(path):
    with zipfile.ZipFile(path) as z:
        ts    = np.load(z.open("timestamps.npy"))        # rosbag sim_time [s]
        e     = np.load(z.open("error_array.npy"))
        stats = json.loads(z.read("stats.json"))
    return ts, e, stats

t_base, e_base, s_base = load_zip(BASELINE_ZIP)
t_filt, e_filt, s_filt = load_zip(FILTERED_ZIP)

# ── dynamic event annotations in rosbag simulation time [s] ───────────────────
# The APE curves start at sim_time ≈ 7 s (first bag frame), so the 0–7 s gap
# on the left is intentionally empty – matching the "no pose yet" period.
EVENTS = [
    (  0,  40, "#E74C3C", "Cross-cut ×5–6",    False),
    ( 45,  48, "#E67E22", "Head-on ①",          True),
    ( 55,  59, "#F1C40F", "Head-on ②",          True),
    (131, 158, "#1ABC9C", "Co-dir. walk",        False),
    (159, 170, "#9B59B6", "4-person static",     True),
    (170, 175, "#6C3483", "4-person co-walk",    True),
]

# ── improvement stats ─────────────────────────────────────────────────────────
rmse_imp = (s_base["rmse"] - s_filt["rmse"]) / s_base["rmse"] * 100
mean_imp = (s_base["mean"] - s_filt["mean"]) / s_base["mean"] * 100
max_imp  = (s_base["max"]  - s_filt["max"])  / s_base["max"]  * 100

# ── style ─────────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family":       "DejaVu Sans",
    "font.size":         11,
    "axes.spines.top":   False,
    "axes.spines.right": False,
    "figure.dpi":        150,
})

fig, ax = plt.subplots(figsize=(13, 4.8))
fig.patch.set_facecolor("#F8F9FA")
ax.set_facecolor("#F8F9FA")

# ── determine y-limit first so shading + labels are consistent ────────────────
y_max = max(e_base.max(), e_filt.max()) * 1.08
ax.set_ylim(0, y_max)
# x-axis spans from 0 (before bag starts) to end of recording
ax.set_xlim(0, t_base[-1])

# ── event shading + labels ────────────────────────────────────────────────────
# All labels are placed at a uniform y band [LABEL_Y_LO, LABEL_Y_HI] in data
# coordinates, centred vertically so the text sits cleanly in that stripe.
LABEL_Y_LO, LABEL_Y_HI = 0.15, 0.20
label_y_ctr = (LABEL_Y_LO + LABEL_Y_HI) / 2   # = 0.175 m

for x0, x1, color, label, rotate in EVENTS:
    ax.axvspan(x0, x1, alpha=0.15, color=color, linewidth=0, zorder=1)
    mid = (x0 + x1) / 2
    ax.text(mid, label_y_ctr, label,
            ha="center", va="center",
            fontsize=8, color=color, fontweight="bold",
            rotation=90 if rotate else 0,
            zorder=5,
            bbox=dict(boxstyle="square,pad=0", fc="none", ec="none"))

# ── APE curves ────────────────────────────────────────────────────────────────
ax.plot(t_base, e_base, color="#E74C3C", lw=1.1, alpha=0.80,
        label=f"Baseline  RMSE = {s_base['rmse']:.4f} m", zorder=3)
ax.plot(t_filt, e_filt, color="#27AE60", lw=1.4,
        label=f"Filtered   RMSE = {s_filt['rmse']:.4f} m", zorder=4)

# RMSE reference lines
ax.axhline(s_base["rmse"], color="#E74C3C", lw=0.9, ls="--", alpha=0.55, zorder=2)
ax.axhline(s_filt["rmse"], color="#27AE60", lw=0.9, ls="--", alpha=0.55, zorder=2)

# ── improvement box ───────────────────────────────────────────────────────────
box_text = (
    f"Improvement vs Baseline\n"
    f"  RMSE  ↓ {rmse_imp:.1f}%\n"
    f"  Mean  ↓ {mean_imp:.1f}%\n"
    f"  Max   ↓ {max_imp:.1f}%"
)
ax.text(0.985, 0.97, box_text,
        transform=ax.transAxes,
        ha="right", va="top",
        fontsize=9, family="monospace",
        bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                  edgecolor="#BBBBBB", alpha=0.93),
        zorder=6)

# ── axes decoration ───────────────────────────────────────────────────────────
ax.set_xlabel("Simulation time  (s)", fontsize=11)
ax.set_ylabel("APE translation error  (m)", fontsize=11)
ax.set_title(
    "APE (translation) — dynamic_v3:  Baseline vs Bayesian Scan-Filter",
    fontsize=13, fontweight="bold", pad=8,
)
ax.xaxis.set_major_locator(MultipleLocator(30))
ax.xaxis.set_minor_locator(MultipleLocator(10))
ax.yaxis.set_major_locator(MultipleLocator(0.05))
ax.grid(axis="y", color="#DDDDDD", lw=0.7, zorder=0)
ax.grid(axis="x", color="#E8E8E8", lw=0.5, zorder=0)

# ── combined legend ───────────────────────────────────────────────────────────
event_patches = [
    mpatches.Patch(facecolor=c, alpha=0.5, label=lbl)
    for _, _, c, lbl, _ in EVENTS
]
line_handles, line_labels = ax.get_legend_handles_labels()
ax.legend(
    handles=line_handles + event_patches,
    labels=line_labels + [p.get_label() for p in event_patches],
    loc="upper left", fontsize=8.5, framealpha=0.92,
    ncol=2, columnspacing=0.8, handlelength=1.4, borderpad=0.6,
)

fig.tight_layout(pad=1.0)

# ── save ──────────────────────────────────────────────────────────────────────
for ext in ("png", "pdf"):
    out = OUT_DIR / f"ape_comparison_dynamic_v3.{ext}"
    fig.savefig(out, dpi=150, bbox_inches="tight",
                facecolor=fig.get_facecolor())
    print(f"Saved → {out}")

# also write the PNG to docs/assets/ so it is git-tracked and README can reference it
docs_out = DOCS_DIR / "ape_comparison_dynamic_v3.png"
fig.savefig(docs_out, dpi=150, bbox_inches="tight",
            facecolor=fig.get_facecolor())
print(f"Saved → {docs_out}  (git-tracked copy)")

plt.close(fig)
