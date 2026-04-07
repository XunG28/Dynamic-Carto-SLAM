"""
APE Comparison Plot: static_v2 — clean odom vs slip vs KISS-ICP+EKF fused

Usage:
    python3 scripts/plot_ape_static_v2.py
Input:
    eval/plots/static_v2/ape_{odom,noisy,fused}.zip
Output:
    eval/plots/ape_comparison_static_v2.{png,pdf}
    docs/assets/ape_comparison_static_v2.png   (git-tracked, for README)
"""

import zipfile, json, pathlib
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import MultipleLocator

REPO_ROOT = pathlib.Path(__file__).parent.parent
OUT_DIR   = REPO_ROOT / "eval/plots"
DOCS_DIR  = REPO_ROOT / "docs/assets"
OUT_DIR.mkdir(parents=True, exist_ok=True)
DOCS_DIR.mkdir(parents=True, exist_ok=True)

def load_zip(path):
    with zipfile.ZipFile(path) as z:
        ts    = np.load(z.open("timestamps.npy"))
        e     = np.load(z.open("error_array.npy"))
        stats = json.loads(z.read("stats.json"))
    return ts, e, stats

t_odom,  e_odom,  s_odom  = load_zip(REPO_ROOT / "eval/plots/static_v2/ape_odom.zip")
t_noisy, e_noisy, s_noisy = load_zip(REPO_ROOT / "eval/plots/static_v2/ape_noisy.zip")
t_fused, e_fused, s_fused = load_zip(REPO_ROOT / "eval/plots/static_v2/ape_fused.zip")

# ── improvement stats ─────────────────────────────────────────────────────────
rmse_imp = (s_noisy["rmse"] - s_fused["rmse"]) / s_noisy["rmse"] * 100
mean_imp = (s_noisy["mean"] - s_fused["mean"]) / s_noisy["mean"] * 100
max_imp  = (s_noisy["max"]  - s_fused["max"])  / s_noisy["max"]  * 100

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

y_max = max(e_odom.max(), e_noisy.max(), e_fused.max()) * 1.06
ax.set_ylim(0, y_max)
ax.set_xlim(min(t_odom[0], t_noisy[0], t_fused[0]),
            max(t_odom[-1], t_noisy[-1], t_fused[-1]))

# ── APE curves ────────────────────────────────────────────────────────────────
ax.plot(t_noisy, e_noisy, color="#E74C3C", lw=1.0, alpha=0.80,
        label=f"Degraded odom (slip)  RMSE = {s_noisy['rmse']:.4f} m", zorder=3)
ax.plot(t_fused, e_fused, color="#27AE60", lw=1.4,
        label=f"KISS-ICP + EKF fused  RMSE = {s_fused['rmse']:.4f} m", zorder=4)
ax.plot(t_odom,  e_odom,  color="#2980B9", lw=1.0, alpha=0.75, ls="--",
        label=f"Clean odom (baseline) RMSE = {s_odom['rmse']:.4f} m",  zorder=2)

# RMSE reference lines
ax.axhline(s_noisy["rmse"], color="#E74C3C", lw=0.8, ls=":",  alpha=0.55)
ax.axhline(s_fused["rmse"], color="#27AE60", lw=0.8, ls=":",  alpha=0.55)
ax.axhline(s_odom["rmse"],  color="#2980B9", lw=0.8, ls=":",  alpha=0.45)

# ── improvement box ───────────────────────────────────────────────────────────
box_text = (
    f"EKF fusion vs degraded odom\n"
    f"  RMSE  ↓ {rmse_imp:.1f}%\n"
    f"  Mean  ↓ {mean_imp:.1f}%\n"
    f"  Max   ↓ {max_imp:.1f}%"
)
ax.text(0.985, 0.97, box_text,
        transform=ax.transAxes, ha="right", va="top",
        fontsize=9, family="monospace",
        bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                  edgecolor="#BBBBBB", alpha=0.93),
        zorder=6)

# ── axes ──────────────────────────────────────────────────────────────────────
ax.set_xlabel("Simulation time  (s)", fontsize=11)
ax.set_ylabel("APE translation error  (m)", fontsize=11)
ax.set_title(
    "APE (translation) — static_v2:  Clean odom vs Slip vs KISS-ICP+EKF Fusion",
    fontsize=13, fontweight="bold", pad=8,
)
ax.xaxis.set_major_locator(MultipleLocator(30))
ax.xaxis.set_minor_locator(MultipleLocator(10))
ax.yaxis.set_major_locator(MultipleLocator(0.1))
ax.grid(axis="y", color="#DDDDDD", lw=0.7, zorder=0)
ax.grid(axis="x", color="#E8E8E8", lw=0.5, zorder=0)
ax.legend(loc="upper left", fontsize=9, framealpha=0.92,
          handlelength=1.5, borderpad=0.6)

fig.tight_layout(pad=1.0)

# ── save ──────────────────────────────────────────────────────────────────────
for ext in ("png", "pdf"):
    out = OUT_DIR / f"ape_comparison_static_v2.{ext}"
    fig.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"Saved → {out}")

docs_out = DOCS_DIR / "ape_comparison_static_v2.png"
fig.savefig(docs_out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
print(f"Saved → {docs_out}  (git-tracked copy)")

plt.close(fig)
