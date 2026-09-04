#!/usr/bin/env python3
"""Plot joint torques from joint_motion_controller.cpp output.

Generates paper-ready plots with light/dark theme support.

Usage:
  python scripts/plotting/plot_joint_torques.py <output.csv> [--output output.svg]
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
    Returns: time, desired positions, joint torques
    """
    with open(csv_path) as f:
        reader = csv.reader(f)
        data = np.array([list(map(float, row)) for row in reader])

    time = data[:, 0]
    tau = data[:, 1 + 2 * ROBOT_DOF : 1 + 3 * ROBOT_DOF]
    return time, tau


def plot_torques(time: np.ndarray, tau: np.ndarray, output_path: str = None) -> None:
    """Plot all joint torques on a single plot with professional color scheme."""
    fig, ax = plt.subplots(figsize=(8, 6))

    # Professional, color-blind friendly palette
    colors = [
        "#1f77b4",  # Blue
        "#ff7f0e",  # Orange
        "#2ca02c",  # Green
        "#d62728",  # Red
        "#9467bd",  # Purple
        "#8c564b",  # Brown
    ]

    for i in range(ROBOT_DOF):
        ax.plot(time, tau[:, i], color=colors[i], linewidth=2, label=JOINT_NAMES[i])

    ax.set_xlabel(r"Time [s]", fontsize=20, fontweight="bold")
    ax.set_ylabel(r"Torque [N$\cdot$m]", fontsize=20, fontweight="bold")
    ax.grid(True, alpha=0.3, linestyle="--", linewidth=0.5)
    ax.tick_params(axis="both", which="major", labelsize=16)
    # Make tick labels bold
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontweight("bold")
    legend = ax.legend(loc="upper right", fontsize=14, framealpha=0.95)
    # Make legend labels bold
    for text in legend.get_texts():
        text.set_fontweight("bold")

    plt.tight_layout()

    if output_path is None:
        output_path = "joint_motion_control_output.svg"

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
      g[id="legend_1"] g[id*="patch"] path {
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
        g[id="legend_1"] g[id*="patch"] path {
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
        default="joint_motion_control_output.svg",
        help="Output SVG file (default: joint_motion_control_output.svg).",
    )
    args = parser.parse_args()

    time, tau = load_csv(args.csv_path)
    plot_torques(time, tau, args.output)


if __name__ == "__main__":
    main()
