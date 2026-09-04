#!/usr/bin/env python3
"""Plot joint position tracking performance (desired vs actual).

Generates paper-ready plots with light/dark theme support for evaluating
the joint motion controller's tracking accuracy.

Usage:
  python scripts/plotting/plot_joint_tracking.py <output.csv> [--output output.svg]
"""

import argparse
import matplotlib.pyplot as plt
import numpy as np
import csv
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

plt.style.use(REPO_ROOT / "scripts/plotting/latex_plot.mplstyle")

ROBOT_DOF = 6
JOINT_NAMES = [f"Joint {i + 1}" for i in range(ROBOT_DOF)]


def load_csv(csv_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load CSV output from joint_motion_controller.

    Expected columns: t, q[0..5], q_d[0..5], tau[0..5]
    Returns: time, actual positions (q), desired positions (q_d)
    """
    with open(csv_path) as f:
        reader = csv.reader(f)
        data = np.array([list(map(float, row)) for row in reader])

    time = data[:, 0]
    q = data[:, 1 : 1 + ROBOT_DOF]  # Actual positions
    q_d = data[:, 1 + ROBOT_DOF : 1 + 2 * ROBOT_DOF]  # Desired positions
    return time, q, q_d


def plot_tracking(time: np.ndarray, q: np.ndarray, q_d: np.ndarray, output_path: str = None) -> None:
    """Plot joint position tracking performance in a 2x3 grid (one subplot per joint)."""
    fig, axes = plt.subplots(3, 2, figsize=(8, 9))
    axes = axes.flatten()

    for i in range(ROBOT_DOF):
        ax = axes[i]
        ax.plot(time, q_d[:, i], "#ff7f0e", linewidth=4.0, label="Desired", alpha=0.8)
        ax.plot(time, q[:, i], "#1f77b4", linewidth=2.0, label="Actual", linestyle="--")

        ax.set_xlabel(r"Time [s]", fontsize=16, fontweight="bold")
        ax.set_ylabel(r"Position [rad]", fontsize=16, fontweight="bold")
        ax.set_title(JOINT_NAMES[i], fontsize=11, fontweight="bold")
        ax.grid(True, alpha=0.3, linestyle="--", linewidth=0.5)
        ax.tick_params(axis="both", which="major", labelsize=10)
        for label in ax.get_xticklabels() + ax.get_yticklabels():
            label.set_fontweight("bold")
        ax.legend(loc="best", fontsize=10, framealpha=0.95)

    plt.tight_layout()

    if output_path is None:
        output_path = "joint_tracking_output.svg"

    # Save as PNG for visualization
    png_path = output_path.replace(".svg", ".png")
    plt.savefig(png_path, format="png", dpi=150, bbox_inches="tight")
    print(f"Saved PNG to {png_path}")

    # Save as SVG
    plt.savefig(output_path, format="svg", dpi=300, bbox_inches="tight")
    print(f"Saved SVG to {output_path}")
    _make_svg_transparent_with_dark_mode(output_path)
    
    plt.show()


def _make_svg_transparent_with_dark_mode(svg_path: str) -> None:
    """Make SVG background transparent and add CSS for light/dark theme backgrounds."""
    with open(svg_path, "r", encoding="utf-8") as f:
        content = f.read()

    # Remove white background fills so Sphinx theme can provide the background
    import re
    
    # Find and remove the white patch rectangles (figure background and axes background)
    # Replace fill: #ffffff with fill: none (transparent)
    content = re.sub(
        r'(path[^>]*style="[^"]*fill: #ffffff[^"]*")',
        lambda m: m.group(1).replace('fill: #ffffff', 'fill: none'),
        content
    )
    
    # Add CSS that provides backgrounds and colors for light/dark modes
    css = '''  <defs>
    <style type="text/css">
      <![CDATA[
      /* Light mode - white background, black text */
      svg {
        background-color: #ffffff;
      }
      g[id*="text"] path,
      g[id*="text"] use {
        fill: black;
      }
      line[style*="stroke: #000000"],
      use[style*="stroke: #000000"],
      path[style*="stroke: #000000"] {
        stroke: black;
      }
      g[id*="legend"] g[id*="patch"] path {
        fill: #ffffff;
        stroke: #cccccc;
      }
      path[style*="stroke: #b0b0b0"] {
        stroke: #b0b0b0;
      }
      
      /* Dark mode - dark background, white text */
      @media (prefers-color-scheme: dark) {
        svg {
          background-color: #121212;
        }
        g[id*="text"] path,
        g[id*="text"] use {
          fill: white;
        }
        line[style*="stroke: #000000"],
        use[style*="stroke: #000000"],
        path[style*="stroke: #000000"] {
          stroke: white;
        }
        g[id*="legend"] g[id*="patch"] path {
          fill: #121212;
          stroke: #444444;
        }
        path[style*="stroke: #b0b0b0"] {
          stroke: #1e1e1e;
        }
      }
      ]]>
    </style>
    </defs>
    '''

    # Insert CSS and defs at the beginning after svg tag
    svg_tag_end = content.find(">", content.find("<svg")) + 1
    content = content[:svg_tag_end] + "\n" + css + content[svg_tag_end:]

    with open(svg_path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"Added transparent background with dark mode CSS to {svg_path}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("csv_path", help="CSV output from joint_motion_controller.")
    parser.add_argument(
        "--output",
        default="joint_tracking_output.svg",
        help="Output SVG file (default: joint_tracking_output.svg).",
    )
    args = parser.parse_args()

    time, q, q_d = load_csv(args.csv_path)
    plot_tracking(time, q, q_d, args.output)


if __name__ == "__main__":
    main()
