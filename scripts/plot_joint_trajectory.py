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

offset = 1
robot_dof = 7
trajectory_in = []
trajectory_out = []

with open('../examples/data/breeder_trajectory_interpolated.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[1+i]))
        trajectory_in.append(q)


with open('../build/examples/output.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[i]))
        trajectory_out.append(q)

trajectory_in_arr = np.array(trajectory_in)
trajectory_out_arr = np.array(trajectory_out)

width = 345
cm = 1/2.54  # centimeters in inches
fig, ax = plt.subplots(figsize=set_size(width), tight_layout=True)
ax2 = ax.twinx()

ax.plot(trajectory_in_arr, label='q_d', linewidth=1)
ax.set_xlabel('samples')
ax.set_ylabel('joint positions [rad]')
ax2.plot(trajectory_out_arr, '--', label='q_out', linewidth=1)


fig.legend(['$q_d[0]$', '$q_d[1]$', '$q_d[2]$', '$q_d[3]$', '$q_d[4]$', '$q_d[5]$', '$q_d[6]$', '$q_{out}[0]$', '$q_{out}[1]$', '$q_{out}[2]$', '$q_{out}[3]$', '$q_{out}[4]$', '$q_{out}[5]$', '$q_{out}[6]$'], loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax.transAxes)
plt.title('Joint PD Controller Output')
plt.savefig('joint_pd_control_output.pdf')
plt.show()

