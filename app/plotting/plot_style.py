from __future__ import annotations
import matplotlib.pyplot as plt

def apply_thesis_style() -> None:
    """White background, black text, consistent look for thesis figures."""
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "text.color": "black",
        "axes.labelcolor": "black",
        "axes.edgecolor": "black",
        "xtick.color": "black",
        "ytick.color": "black",
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "legend.fontsize": 10,
        "lines.linewidth": 2.0,
        "grid.color": "#cccccc",
        "grid.linewidth": 0.6,
        "grid.alpha": 0.6,
    })

def save_figure(fig, path_no_ext: str, dpi: int = 300) -> None:
    """Save both PNG and SVG."""
    fig.savefig(path_no_ext + ".png", dpi=dpi, bbox_inches="tight")
    fig.savefig(path_no_ext + ".svg", bbox_inches="tight")
