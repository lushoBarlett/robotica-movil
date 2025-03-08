import numpy as np


def load_estimated_positions(estimated_file: str) -> np.ndarray:
    estimated_positions = []
    with open(estimated_file, "r") as f:
        for line in f.readlines()[4:]:
            parts = line.strip().split()
            if not parts:
                continue
            estimated_positions.append(
                (float(parts[-5]), float(parts[-4]), float(parts[-3]))
            )
    return np.array(estimated_positions)


def load_ref_positions(ref_file: str) -> np.ndarray:
    ref_positions = []
    with open(ref_file, "r") as f:
        for line in f.readlines():
            parts = line.strip().split()
            if not parts:
                continue
            ref_positions.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return np.array(ref_positions)
