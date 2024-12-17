import matplotlib.pyplot as plt
import numpy as np
import csv
import matplotlib as mpl
mpl.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
    # Use 10pt font in plots, to match 10pt font in document
    "axes.labelsize": 10,
    "font.size": 10,
    # Make the legend/label fonts a little smaller
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8
})

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
#plt.style.use('latex_plot.mplstyle')

robot_dof = 7
time = []
trajectory_in = []
trajectory_out = []

with open('../examples/data/joint_trajectory_safe_bb.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        # read time
        time.append(float(row[0]))
        # read q
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[1+i]))
        trajectory_in.append(q)

time.pop()
trajectory_in.pop()

with open('../build/examples/output_eurofusion_joint.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[i]))
        trajectory_out.append(q)

trajectory_in_arr = np.array(trajectory_in)
trajectory_out_arr = np.array(trajectory_out)
time_arr = np.array(time)

width = 345
cm = 1/2.54  # centimeters in inches
fig, ax = plt.subplots(figsize=set_size(width), dpi=300) #
ax2 = ax.twinx()
plt.subplots_adjust(top=0.845,
                    bottom=0.2,
                    left=0.2,
                    right=0.96,
                    hspace=0.2,
                    wspace=0.2)
print(len(trajectory_in_arr))
print(len(trajectory_out_arr))
ax.plot(time_arr, trajectory_in_arr, label='q_d', linewidth=1)
ax.set_xlabel('Time [s]')
ax.set_ylabel('Joint positions [rad]')
ax2.plot(time_arr, trajectory_out_arr, '--', label='q_out', linewidth=1)
fig.legend(['$q_d[0]$', '$q_d[1]$', '$q_d[2]$', '$q_d[3]$', '$q_d[4]$', '$q_d[5]$', '$q_d[6]$', '$q_{out}[0]$', '$q_{out}[1]$', '$q_{out}[2]$', '$q_{out}[3]$', '$q_{out}[4]$', '$q_{out}[5]$', '$q_{out}[6]$'], loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax.transAxes)
plt.savefig('joint_pd_control_output_bb.pdf', bbox_inches='tight')
plt.show()

