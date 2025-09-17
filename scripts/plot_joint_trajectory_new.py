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
robot_dof = 6
trajectory_in = []
trajectory_out = []

with open('../data/joint_trajectory_safe_new.csv') as csv_file_in:
    csv_reader_in = csv.reader(csv_file_in, delimiter=',')
    line_count = 0
    for row in csv_reader_in:
        q = []
        for i in range(0, robot_dof):
            q.append(float(row[i]))
        #print('qd:', q)
        trajectory_in.append(q)


#with open('../build/examples/hardware/ur/output.csv') as csv_file:
#    csv_reader = csv.reader(csv_file, delimiter=',')
#    line_count = 0
#    for row in csv_reader:
#        q = []
#        for i in range(1, robot_dof+1):
#            q.append(float(row[i]))

        #print('qout:', q)
#        trajectory_out.append(q)

trajectory_in_arr = np.array(trajectory_in)
#trajectory_out_arr = np.array(trajectory_out)

width = 345
cm = 1/2.54  # centimeters in inches
fig, ax = plt.subplots(figsize=set_size(width), tight_layout=True)
ax.plot(trajectory_in_arr, label='q_d', linewidth=1)
#ax.plot(trajectory_out_arr, '--', label='q_out', linewidth=1)
ax.set_xlabel('samples')
ax.set_ylabel('joint positions [rad]')

fig.legend(['$q_d[0]$', '$q_d[1]$', '$q_d[2]$', '$q_d[3]$', '$q_d[4]$', '$q_d[5]$'], loc="upper right", bbox_to_anchor=(1, 1))
plt.title('Joint PD Controller Output')
#plt.savefig('joint_pd_control_output.pdf')
plt.show()
