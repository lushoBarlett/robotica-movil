import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # type: ignore


def plot_3d_points(points1: np.ndarray, points2: np.ndarray, filename: str) -> None:
    """Plot two sets of 3D points. Indicating what the first point of each set is."""
    fig = plt.figure()
    ax: Axes3D = fig.add_subplot(111, projection="3d")

    ax.scatter(
        points1[:, 0],
        points1[:, 1],
        points1[:, 2],
        c="b",
        label="Estimated",
        s=10,
    )
    ax.scatter(
        points2[:, 0], points2[:, 1], points2[:, 2], c="r", label="Reference", s=10
    )

    ax.text(points1[0, 0], points1[0, 1], points1[0, 2], "Start", color="b")
    ax.text(points2[0, 0], points2[0, 1], points2[0, 2], "Start", color="r")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()

    plt.savefig(f"plots/{filename}")
