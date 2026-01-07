import csv
import matplotlib.pyplot as plt

xs, ys = [], []
xg, yg = [], []

with open("lab2_traj_log.csv", newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        xs.append(float(row['x_est']))
        ys.append(float(row['y_est']))
        xg.append(float(row['x_gt']))
        yg.append(float(row['y_gt']))
        tx,ty = 4.0, -1.0


plt.figure()
plt.scatter([tx], [ty], marker='x', s=70, label='Target')
plt.plot(xs, ys, label='Estimated trajectory')
plt.plot(xg, yg, '--', label='Ground truth')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.axis('equal')
plt.legend()
plt.grid()
plt.title('GoToTarget trajectory')
plt.show()
