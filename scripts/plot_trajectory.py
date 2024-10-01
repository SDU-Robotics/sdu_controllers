import matplotlib.pyplot as plt
import numpy as np
import csv

trajectory_in = []
trajectory_out = []

with open('../examples/data/trajectory.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, 6):
            q.append(float(row[i]))
        trajectory_in.append(q)


with open('../build/examples/output.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        q = []
        for i in range(0, 6):
            q.append(float(row[i]))
        trajectory_out.append(q)

trajectory_in_arr = np.array(trajectory_in)
trajectory_out_arr = np.array(trajectory_out)

fig, ax = plt.subplots()
ax2 = ax.twinx()

ax.plot(trajectory_in_arr, label='q_d')
ax.set_xlabel('samples')
ax.set_ylabel('joint position [rad]')
ax2.plot(trajectory_out_arr, '--', label='q_out')
fig.legend(['q_d[0]', 'q_d[1]', 'q_d[2]', 'q_d[3]', 'q_d[4]', 'q_d[5]', 'q_out[0]', 'q_out[1]', 'q_out[2]',
            'q_out[3]', 'q_out[4]', 'q_out[5]'], loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax.transAxes)
plt.title('Joint PD Controller Output')
plt.show()
