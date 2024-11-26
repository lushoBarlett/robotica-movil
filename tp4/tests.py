import os

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

    with open("pfext.txt", "w") as f:
        f.write("")

    particles = [20, 50, 500]
    for i in r:
        for n in particles:
            print(f"Running 10 runs for r = {i} and n = {n}")
            for _ in range(10):
                command = f"python localization.py pf --data-factor {i} --filter-factor {i} --num-particles {n} >> pfext.txt"
                print("$", command)
                os.system(f"python localization.py pf --data-factor {i} --filter-factor {i} --num-particles {n} >> pfext.txt")

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
    import matplotlib.pyplot as plt

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

    if log:
        plt.xscale("log")
    else:
        plt.xscale("linear")

    plt.plot(x, y)
    plt.xlabel("r")
    plt.ylabel("Mean position error")
    plt.show()
    plt.savefig(prefix + "mean_position_error.png")

    if log:
        plt.xscale("log")
    else:
        plt.xscale("linear")

    plt.plot(x, y, label="Mean position error")
    plt.plot(x, y2, label="ANEES")
    plt.xlabel("r")
    plt.legend()
    plt.show()
    plt.savefig(prefix + "mean_position_error_and_anees.png")

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
    # used initially
    # test_ekf()
    # test_pf()
    # test_pf_extensive()
    # plot_runs_from_file("ekf.txt")
    # plot_runs_from_file("pf.txt")
    plot_runs_from_file("pfext.txt", log=True)
