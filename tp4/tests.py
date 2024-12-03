import os
import matplotlib.pyplot as plt
import numpy as np

r = [1/64, 1/16, 1/4, 4, 16, 64]

def test_ekf():
    print("Running EKF tests")

    with open("ekf.txt", "w") as f:
        f.write("")

    for i in r:
        print(f"Running 10 runs for r = {i}")
        for _ in range(10):
            command = f"python localization.py ekf --data-factor {i} --filter-factor {i} >> ekf.txt"
            print("$", command)
            os.system(f"python localization.py ekf --data-factor {i} --filter-factor {i} >> ekf.txt")

def test_pf():
    print("Running PF tests")

    with open("pf.txt", "w") as f:
        f.write("")

    for i in r:
        print(f"Running 10 runs for r = {i}")
        for _ in range(10):
            command = f"python localization.py pf --data-factor {i} --filter-factor {i} >> pf.txt"
            print("$", command)
            os.system(f"python localization.py pf --data-factor {i} --filter-factor {i} >> pf.txt")

def test_pf_extensive():
    print("Running PF extensive tests")

    particles = [20, 50, 500]

    for n in particles:
        with open(f"pfext{n}.txt", "w") as f:
            f.write("")

        for i in r:
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

"""
Graficar el error de posición medio a medida que los factores α, β varían sobre r y discuta los
resultados.

Graficar el error de posición medio y ANEES a medida que los factores α, β del filtro varían sobre
r mientras los datos son generados con los valores por defecto.
"""
def plot(runs: list[Run], prefix: str = "", log=False):
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

    plt.figure(figsize=(10, 6), dpi=120)
    plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.7)

    if log:
        plt.xscale("log")
    else:
        plt.xscale("linear")

    normalized = [i / (max(y) - min(y)) for i in y]

    fig, ax1 = plt.subplots()
    ax1.plot(x, y)
    ax2 = ax1.twinx()
    ax2.plot(x, normalized, color="red")

    plt.plot(x, y)
    plt.xlabel("r")
    plt.ylabel("Mean position error")
    plt.savefig(prefix + "mean_position_error.png")
    plt.show()

    plt.figure(figsize=(10, 6), dpi=120)
    plt.grid(color='gray', linestyle='--', linewidth=0.5, alpha=0.7)

    if log:
        plt.xscale("log")
    else:
        plt.xscale("linear")

    plt.plot(x, y, label="Mean position error")
    plt.plot(x, y2, label="ANEES")
    plt.xlabel("r")
    plt.legend()
    plt.savefig(prefix + "mean_position_error_and_anees.png")
    plt.show()

def plot_normalized(x, ys):
    normalized = [(y - np.min(y)) / (np.max(y) - np.min(y)) for y in ys]

    colors = plt.cm.tab10(np.linspace(0, 1, len(ys)))

    plt.figure(figsize=(10, 6))
    for i, n in enumerate(normalized):
        color = colors[i]

        plt.plot(x, n, label=f'Line {i + 1}', color=color)

        min_index = np.argmin(n)
        max_index = np.argmax(n)

        plt.scatter(x[min_index], 0, color=color, zorder=5)  # Min point
        plt.scatter(x[max_index], 1, color=color, zorder=5)  # Max point

        plt.annotate(
            f"Min: {ys[i][min_index]:.2f}",
            (x[min_index], 0),
            textcoords="offset points",
            xytext=(-40, 5),
            ha='center',
            color=color
        )

        plt.annotate(
            f"Max: {ys[i][max_index]:.2f}",
            (x[max_index], 1),
            textcoords="offset points",
            xytext=(40, -15),
            ha='center',
            color=color
        )

    plt.gca().axes.yaxis.set_visible(False)
    plt.grid(color='gray', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.show()

def plot_runs_from_file(filename: str, log=False):
    with open(filename, "r") as f:
        lines = f.readlines()

        runs = []
        while len(lines) > 0:
            run, lines = parse_run(lines)
            if run:
                runs.append(run)

        plot(runs, filename.split(".")[0] + "_", log)

if __name__ == "__main__":
    # used in the first run because it takes a long time
    # test_ekf()
    # test_pf()
    # test_pf_extensive()

    plot_runs_from_file("ekf.txt", log=True)
    plot_runs_from_file("pf.txt", log=True)

    for n in [20, 50, 500]:
        plot_runs_from_file(f"pfext{n}.txt", log=True)
