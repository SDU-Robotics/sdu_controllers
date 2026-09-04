import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[2]

ref_traj = []
adm_traj = []

with open(REPO_ROOT / 'build/examples/output_admittance_reference.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        ref_traj.append([float(row[0]), float(row[1])])

with open(REPO_ROOT / 'build/examples/output_admittance.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        adm_traj.append([float(row[0]), float(row[1])])


ref_traj = np.array(ref_traj)
adm_traj = np.array(adm_traj)

plt.figure(figsize=(8, 8))
plt.plot(ref_traj[:, 0], ref_traj[:, 1], label="desired trajectory", color='red')
plt.plot(adm_traj[:, 0], adm_traj[:, 1], label="admittance trajectory", color='blue')
plt.legend()
plt.show()