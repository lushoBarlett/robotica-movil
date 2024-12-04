import os
import matplotlib.pyplot as plt
import numpy as np


R = [1/64, 1/16, 1/4, 4, 16, 64]


# make each pair of colors very close to each other
COLORS = ["#2f6794", "#7f8ff0", "#4c902c", "#20f070", "#a62728", "#f99098"]


PARTICLES = [20, 50, 500]


def test_ekf():
    print("Running EKF tests")

    with open("ekf.txt", "w") as f:
        f.write("")

    for i in R:
        print(f"Running 10 runs for r = {i}")
        for _ in range(10):
            command = f"python localization.py ekf --data-factor {i} --filter-factor {i} >> ekf.txt"
            print("$", command)
            os.system(f"python localization.py ekf --data-factor {i} --filter-factor {i} >> ekf.txt")


def test_pf():
    print("Running PF tests")

    with open("pf.txt", "w") as f:
        f.write("")

    for i in R:
        print(f"Running 10 runs for r = {i}")
        for _ in range(10):
            command = f"python localization.py pf --data-factor {i} --filter-factor {i} >> pf.txt"
            print("$", command)
            os.system(f"python localization.py pf --data-factor {i} --filter-factor {i} >> pf.txt")


def test_pf_extensive():
    print("Running PF extensive tests")

    for n in PARTICLES:
        with open(f"pfext{n}.txt", "w") as f:
            f.write("")

        for i in R:
            print(f"Running 10 runs for r = {i} and n = {n}")
            for _ in range(10):
                command = f"python localization.py pf --data-factor {i} --filter-factor {i} --num-particles {n} >> pfext{n}.txt"
                print("$", command)
                os.system(f"python localization.py pf --data-factor {i} --filter-factor {i} --num-particles {n} >> pfext{n}.txt")


"""
Example output of one run:
Data factor: {r}
Filter factor: {r}
--------------------------------------------------------------------------------
Mean position error: {mean error}
Mean Mahalanobis error: {mean mahalanobis}
ANEES: {anees}
"""
class Run:
    def __init__(self, data_factor, filter_factor, mean_error, mean_mahalanobis, anees):
        self.data_factor = data_factor
        self.filter_factor = filter_factor
        self.mean_error = mean_error
        self.mean_mahalanobis = mean_mahalanobis
        self.anees = anees

    def __repr__(self):
        return f"Run(data_factor={self.data_factor}, filter_factor={self.filter_factor}, mean_error={self.mean_error}, mean_mahalanobis={self.mean_mahalanobis}, anees={self.anees})"


def parse_run(lines: list[str]):

    if (len(lines) < 6):
        return None, lines

    data_factor = float(lines[0].split(" ")[-1])
    filter_factor = float(lines[1].split(" ")[-1])
    mean_error = float(lines[3].split(":")[-1].strip())
    mean_mahalanobis = float(lines[4].split(":")[-1].strip())
    anees = float(lines[5].split(":")[-1].strip())
    return Run(data_factor, filter_factor, mean_error, mean_mahalanobis, anees), lines[6:]


def style_normalized(x, ys, names):
    normalized = [(y - np.min(y)) / (np.max(y) - np.min(y)) for y in ys]

    for i, (y, n, p, c) in enumerate(zip(ys, normalized, names, COLORS)):
        plt.plot(x, n, label=p, color=c)

        plt.xticks(x, map(lambda x: f"1 / {int(1 / x)}" if 0 < x < 1 else "0" if x == 0 else str(int(x)), x))

        min_index = np.argmin(n)
        max_index = np.argmax(n)

        plt.scatter(x[min_index], 0, color=c, zorder=5)  # Min point
        plt.scatter(x[max_index], 1, color=c, zorder=5)  # Max point

        plt.annotate(
            f"Min: {y[min_index]:.2f}",
            (x[min_index], 0),
            textcoords="offset points",
            xytext=(-40, 5 + i * 10),
            ha='center',
            color=c
        )

        plt.annotate(
            f"Max: {y[max_index]:.2f}",
            (x[max_index], 1),
            textcoords="offset points",
            xytext=(40, -15 - i * 10),
            ha='center',
            color=c
        )

    plt.gca().axes.yaxis.set_visible(False)
    plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.7)
    plt.tight_layout()
    plt.legend()


def plot_data_from_runs(runs: list[Run]) -> tuple[list[float], list[float], list[float]]:
    x = sorted(set(run.data_factor for run in runs))
    y = []; y2 = []
    for r in x:
        runs_r = [run for run in runs if run.data_factor == r]

        mean_errors = [run.mean_error for run in runs_r]
        avg_mean_error = sum(mean_errors) / len(mean_errors)

        y.append(avg_mean_error)

        anees = [run.anees for run in runs_r]
        avg_anees = sum(anees) / len(anees)

        y2.append(avg_anees)

    return x, y, y2


"""
Graficar el error de posición medio a medida que los factores α, β varían sobre r y discuta los
resultados.

Graficar el error de posición medio y ANEES a medida que los factores α, β del filtro varían sobre
r mientras los datos son generados con los valores por defecto.
"""
def plot(runs: list[Run], prefix: str = "", log=False, title=None):
    x, y, y2 = plot_data_from_runs(runs)

    plt.figure(figsize=(10, 6), dpi=120)
    plt.title(title)
    plt.xscale("log" if log else "linear")
    plt.xlabel("r" + (" (log scale)" if log else ""))
    style_normalized(x, [y], ["Mean position error (normalized)"])
    plt.savefig(prefix + "mean_position_error.png")
    plt.show()

    plt.figure(figsize=(10, 6), dpi=120)
    plt.title(title)
    plt.xscale("log" if log else "linear")
    plt.xlabel("r" + (" (log scale)" if log else ""))
    style_normalized(x, [y, y2], ["Mean position error (normalized)", "ANEES (normalized)"])
    plt.savefig(prefix + "mean_position_error_and_anees.png")
    plt.show()


def read_run(filename: str):
    with open(filename, "r") as f:
        lines = f.readlines()

        runs = []
        while len(lines) > 0:
            run, lines = parse_run(lines)
            if run:
                runs.append(run)

        return runs


if __name__ == "__main__":
    # used in the first run because it takes a long time
    # test_ekf()
    # test_pf()
    # test_pf_extensive()

    plot(read_run("ekf.txt"), "ekf_", log=True, title="Extended Kalman Filter")
    plot(read_run("pf.txt"), "pf_", log=True, title="Particle Filter")

    plt.figure(figsize=(10, 6), dpi=120)
    plt.title("Particle Filter with different number of particles")
    plt.xscale("log")
    plt.xlabel("r (log scale)")

    ys = []
    names = []
    for n in PARTICLES:
        xaxis, y, y2 = plot_data_from_runs(read_run(f"pfext{n}.txt"))
        ys.append(y)
        ys.append(y2)
        names.append(f"Mean position error (normalized) for {n} particles")
        names.append(f"ANEES (normalized) for {n} particles")

    style_normalized(xaxis, ys, names)
    plt.savefig("pf_mean_position_error_and_anees_many_particles.png")
    plt.show()
