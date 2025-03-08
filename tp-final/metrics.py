import numpy as np


def compute_mean_ape(P: np.ndarray, Q: np.ndarray) -> float:
    """Compute the mean Absolute Position Error (APE) between two sets of points."""
    return np.mean(np.linalg.norm(P - Q, axis=1))


def compute_auc_score(
    P: np.ndarray, Q: np.ndarray, max_threshold: float = 0.5, num_bins: int = 100
) -> float:
    """Compute the Area Under the Curve (AUC) score between two sets of points."""
    errors = np.linalg.norm(P - Q, axis=1)

    thresholds = np.linspace(0, max_threshold, num_bins)
    cdf = np.array([(errors <= t).mean() for t in thresholds])

    auc_score = np.trapz(cdf, thresholds) / max_threshold

    return auc_score
