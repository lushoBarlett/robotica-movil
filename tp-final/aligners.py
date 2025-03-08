from typing import Tuple

import cv2
import numpy as np


def ransac_aligner(
    points1: np.ndarray, points2: np.ndarray, ransac_threshold: float = 1.0
) -> Tuple[np.ndarray, np.ndarray]:
    """Align two sets of 3D points using the RANSAC algorithm."""
    assert (
        len(points1) >= 3
    ), "Need at least 3 corresponding points for 3D affine transformation!"

    # Estimate transformation using RANSAC
    retval, M, inliers = cv2.estimateAffine3D(
        points1, points2, ransacThreshold=ransac_threshold
    )

    if not retval:
        raise ValueError("RANSAC failed to converge!")

    points1_transformed = np.hstack((points1, np.ones((points1.shape[0], 1)))) @ M.T

    inlier_mask = inliers.flatten().astype(bool)

    return points1_transformed, inlier_mask


def kabsch_aligner(points1: np.ndarray, points2: np.ndarray) -> np.ndarray:
    """Align two sets of 3D points using the Kabsch algorithm."""
    centroid_P = np.mean(points1, axis=0)
    centroid_Q = np.mean(points2, axis=0)

    P_centered = points1 - centroid_P
    Q_centered = points2 - centroid_Q

    H = P_centered.T @ Q_centered

    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = centroid_Q - R @ centroid_P

    P_transformed = (R @ points1.T).T + t

    return P_transformed


def umeyama_aligner(
    points1: np.ndarray, points2: np.ndarray
) -> Tuple[np.ndarray, float]:
    """Align two sets of 3D points using the Umeyama algorithm (scaling + rotation + translation)."""
    centroid_P = np.mean(points1, axis=0)
    centroid_Q = np.mean(points2, axis=0)

    P_centered = points1 - centroid_P
    Q_centered = points2 - centroid_Q

    H = P_centered.T @ Q_centered

    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    sigma_P = np.mean(np.linalg.norm(P_centered, axis=1))
    sigma_Q = np.mean(np.linalg.norm(Q_centered, axis=1))
    s = sigma_Q / sigma_P

    t = centroid_Q - s * R @ centroid_P

    P_transformed = (s * R @ points1.T).T + t

    return P_transformed, s
