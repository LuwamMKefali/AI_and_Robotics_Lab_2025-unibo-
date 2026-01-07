import os
import numpy as np
import matplotlib.pyplot as plt


def plot_from_csv(csv_path: str, tag: str):
    if not os.path.exists(csv_path):
        print(f"[WARN] File not found: {csv_path} (skipping)")
        return

    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)

    x_est, y_est, th_est, x_gt, y_gt, th_gt, err = data.T

    # Trajectory plot
    plt.figure()
    plt.plot(x_gt, y_gt, label="Ground truth")
    plt.plot(x_est, y_est, label="Odometry estimate")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(f"Robot trajectory: odometry vs ground truth ({tag})")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.savefig(f"trajectory_comparison_{tag}.pdf")
    plt.savefig(f"trajectory_comparison_{tag}.png")
    plt.show()

    # Error plot
    plt.figure()
    plt.plot(err)
    plt.xlabel("time step")
    plt.ylabel("position error [m]")
    plt.title(f"Odometry position error over time ({tag})")
    plt.grid(True)
    plt.savefig(f"position_error_{tag}.pdf")
    plt.savefig(f"position_error_{tag}.png")
    plt.show()

    print(f"[OK] Plots saved for {tag} from {csv_path}")


plot_from_csv("lab1_traj_good.csv", "good")
plot_from_csv("lab1_traj_bad.csv", "bad")
