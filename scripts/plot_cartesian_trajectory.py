import matplotlib.pyplot as plt
import numpy as np
import csv
def set_size(width, fraction=1.5):
    """Set figure dimensions to avoid scaling in LaTeX.

    Parameters
    ----------
    width: float
            Document textwidth or columnwidth in pts
    fraction: float, optional
            Fraction of the width which you wish the figure to occupy

    Returns
    -------
    fig_dim: tuple
            Dimensions of figure in inches
    """
    # Width of figure (in pts)
    fig_width_pt = width * fraction

    # Convert from pt to inches
    inches_per_pt = 1 / 72.27

    # Golden ratio to set aesthetic figure height
    # https://disq.us/p/2940ij3
    golden_ratio = (5**.5 - 1) / 2

    # Figure width in inches
    fig_width_in = fig_width_pt * inches_per_pt
    # Figure height in inches
    fig_height_in = fig_width_in * golden_ratio

    fig_dim = (fig_width_in, fig_height_in)

    return fig_dim

from sympy.printing.pretty.pretty_symbology import line_width
plt.style.use('latex_plot.mplstyle')

trajectory_in = []
trajectory_out = []

with open('../examples/data/cartesian_trajectory_safe.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        pos = []
        for i in range(0, 6):
            pos.append(float(row[i]))
        trajectory_in.append(pos)

with open('../build/examples/output_cartesian.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        pos = []
        for i in range(6, 12):
            pos.append(float(row[i]))
        trajectory_out.append(pos)

trajectory_in_arr = np.array(trajectory_in)
trajectory_out_arr = np.array(trajectory_out)

width = 345
cm = 1/2.54  # centimeters in inches
fig, ax = plt.subplots(figsize=set_size(width), tight_layout=True)
ax2 = ax.twinx()

ax.plot(trajectory_in_arr, label='x_d', linewidth=1)
ax.set_xlabel('samples')
ax.set_ylabel('TCP pose')
ax2.plot(trajectory_out_arr, '--', label='x_out', linewidth=1)


fig.legend(['x_d[0]', 'x_d[1]', 'x_d[2]', 'x_d[3]', 'x_d[4]', 'x_d[5]', 'x_out[0]', 'x_out[1]', 'x_out[2]', 'x_out[3]', 'x_out[4]', 'x_out[5]'], loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax.transAxes)
plt.title('Cartesian motion controller output')
plt.savefig('cartesian_control_output.pdf')
plt.show()

