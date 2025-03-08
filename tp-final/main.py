import argparse
from typing import Optional, Tuple

import numpy as np
from aligners import kabsch_aligner, ransac_aligner, umeyama_aligner
from load_positions import load_estimated_positions, load_ref_positions
from metrics import compute_auc_score, compute_mean_ape
from plot import plot_3d_points


def align_points(
    points1: np.ndarray, points2: np.ndarray, alignment_method: str
) -> Tuple[np.ndarray, np.ndarray, Optional[str]]:
    if alignment_method == "ransac_inliers":
        points1_transformed, inlier_mask = ransac_aligner(points1, points2)
        points1_transformed = points1_transformed[inlier_mask]
        points2 = points2[inlier_mask]

        extra_message = f"RANSAC inlier ratio: {inlier_mask.mean():.4f}"
        return points1_transformed, points2, extra_message

    elif alignment_method == "ransac_all":
        points1_transformed, _ = ransac_aligner(points1, points2)

        return points1_transformed, points2, None

    elif alignment_method == "kabsch":
        points1_transformed = kabsch_aligner(points1, points2)

        return points1_transformed, points2, None

    elif alignment_method == "umeyama":
        points1_transformed, scaling_factor = umeyama_aligner(points1, points2)

        extra_message = f"Scaling factor: {scaling_factor:.4f}"
        return points1_transformed, points2, extra_message

    else:
        raise ValueError(f"Unknown alignment method: {alignment_method}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Process some arguments.")

    parser.add_argument(
        "--dataset",
        "-d",
        type=str,
        choices=["rd", "mh01"],
        help="Dataset to use",
        required=True,
    )
    parser.add_argument(
        "--method",
        type=str,
        choices=["glomap", "colmap"],
        help="Method to use",
        required=True,
    )
    parser.add_argument(
        "--thresholds", type=float, nargs="+", help="List of thresholds", required=True
    )
    parser.add_argument(
        "--alignment",
        type=str,
        choices=["ransac_inliers", "ransac_all", "kabsch", "umeyama"],
        help="Alignment method",
        required=True,
    )
    parser.add_argument(
        "--plot", action="store_true", help="Whether to plot the results"
    )

    args = parser.parse_args()

    estimation_file = f"data/{args.dataset}.{args.method}.txt"
    gt_file = f"data/{args.dataset}.gt.txt"

    points1 = load_estimated_positions(estimation_file)
    points2 = load_ref_positions(gt_file)

    points1, points2, alignment_msg = align_points(points1, points2, args.alignment)

    mean_ape = compute_mean_ape(points1, points2)
    auc_scores = [
        compute_auc_score(points1, points2, max_threshold=t) for t in args.thresholds
    ]

    print("Metrics:")
    print(f"Alignment method: {args.alignment}")

    if alignment_msg is not None:
        print(alignment_msg)

    print(f"Mean APE: {mean_ape:.4f}")
    for i, threshold in enumerate(args.thresholds):
        print(f"AUC score (threshold : {threshold}): {auc_scores[i]:.4f}")

    if args.plot:
        plot_3d_points(points1, points2)


if __name__ == "__main__":
    main()
