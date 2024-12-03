#include "stereoproc.hpp"
#include "cam_params.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

static void undistort_image(cv::Mat &image, const cv::Mat &K, const cv::Mat &D,
                            cv::Mat &out_image) {
    cv::undistort(image, out_image, K, D);
}

static void rectify_images(const cv::Mat &imgLeft, const cv::Mat &imgRight, cv::Mat &rectifiedLeft,
                           cv::Mat &rectifiedRight, cv::Mat &Q, cv::Mat &P_left_rect,
                           cv::Mat &P_right_rect) {
    if (imgLeft.empty() || imgRight.empty()) {
        std::cout << "Error: Could not load images." << std::endl;
        return;
    }

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T,
                              R);

    // Output matrices for rectification
    cv::Mat R1, R2;
    cv::Size imageSize = imgLeft.size();
    cv::stereoRectify(K_left, D_left, K_right, D_right, imageSize, R, T, R1, R2, P_left_rect,
                      P_right_rect, Q);

    // Rectification maps
    cv::Mat map1Left, map2Left, map1Right, map2Right;

    // Compute rectification maps for both images
    cv::initUndistortRectifyMap(K_left, D_left, R1, P_left_rect, imageSize, CV_16SC2, map1Left,
                                map2Left);
    cv::initUndistortRectifyMap(K_right, D_right, R2, P_right_rect, imageSize, CV_16SC2, map1Right,
                                map2Right);

    // Apply rectification
    cv::remap(imgLeft, rectifiedLeft, map1Left, map2Left, cv::INTER_LINEAR);
    cv::remap(imgRight, rectifiedRight, map1Right, map2Right, cv::INTER_LINEAR);
}

static void extract_features(const cv::Mat &imgLeft, const cv::Mat &imgRight,
                             std::vector<cv::KeyPoint> &keypoints_l,
                             std::vector<cv::KeyPoint> &keypoints_r, cv::Mat &descriptors_l,
                             cv::Mat &descriptors_r) {
    int nfeatures = 500;
    float scaleFactor = 1.2f;
    int nlevels = 1;
    int edgeThreshold = 31;
    int firstLevel = 0;
    int WTA_K = 2;
    cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
    int patchSize = 31;
    int fastThreshold = 20;
    cv::Ptr<cv::ORB> orb = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold,
                                           firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

    orb->detect(imgLeft, keypoints_l);
    orb->detect(imgRight, keypoints_r);
    orb->compute(imgLeft, keypoints_l, descriptors_l);
    orb->compute(imgRight, keypoints_r, descriptors_r);
}

static void match_descriptors(cv::Mat &descriptors1, cv::Mat &descriptors2,
                              std::vector<cv::DMatch> &matches) {
    cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true);

    // Match descriptors
    bfMatcher.match(descriptors1, descriptors2, matches);

    // Filter matches by distance threshold
    float distanceThreshold = 30;
    std::vector<cv::DMatch> goodMatches;

    for (const auto &match : matches) {
        if (match.distance < distanceThreshold) {
            goodMatches.push_back(match);
        }
    }
    matches = goodMatches;
}

static void triangulate_points(cv::Mat &P_left, cv::Mat &P_right,
                               std::vector<cv::Point2f> &pointsLeft,
                               std::vector<cv::Point2f> &pointsRight,
                               std::vector<cv::Point3d> &points3D) {
    cv::Mat points4D;
    cv::triangulatePoints(P_left, P_right, pointsLeft, pointsRight, points4D);

    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat x = points4D.col(i);
        x /= x.at<float>(3);

        if (cv::norm(x) > 1000 || x.at<float>(2) <= 0)
            continue;

        points3D.push_back(cv::Point3d(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
    }
}

static void filter_correspondences(std::vector<cv::Point2f> &pointsLeft,
                                   std::vector<cv::Point2f> &pointsRight,
                                   std::vector<cv::DMatch> &matches,
                                   std::vector<cv::Point2f> &transformedPoints,
                                   std::vector<cv::DMatch> &inlierMatches) {
    // Compute homography using RANSAC
    std::vector<uchar> inliersMask(pointsLeft.size());
    cv::Mat H = cv::findHomography(pointsLeft, pointsRight, cv::RANSAC, 3.0, inliersMask);

    // Filter points based on RANSAC inliers
    std::vector<cv::Point2f> filteredPointsLeft, filteredPointsRight;

    // Store filtered points and their corresponding matches
    for (size_t i = 0; i < inliersMask.size(); ++i) {
        if (inliersMask[i]) {
            filteredPointsLeft.push_back(pointsLeft[i]);
            filteredPointsRight.push_back(pointsRight[i]);
            inlierMatches.push_back((matches)[i]);
        }
    }

    // Update the input vectors with inliers
    pointsLeft = std::move(filteredPointsLeft);
    pointsRight = std::move(filteredPointsRight);

    // Transform points from left image to right image using the homography
    cv::perspectiveTransform(pointsLeft, transformedPoints, H);
}

static void compute_disparity_map(cv::Mat &rectifiedLeft, cv::Mat &rectifiedRight,
                                  cv::Mat &disparityMap) {
    cv::Mat disparityMapNotNormalized;
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 9);
    cv::Mat grayLeft, grayRight;
    if (rectifiedLeft.channels() == 1) {
        grayLeft = rectifiedLeft;
        grayRight = rectifiedRight;
    } else {
        cv::cvtColor(rectifiedLeft, grayLeft, cv::COLOR_BGR2GRAY);
        cv::cvtColor(rectifiedRight, grayRight, cv::COLOR_BGR2GRAY);
    }
    stereo->compute(grayLeft, grayRight, disparityMapNotNormalized);

    cv::normalize(disparityMapNotNormalized, disparityMap, 0, 255, cv::NORM_MINMAX, CV_8U);
}

static std::vector<cv::Point3d> dense_reconstruction(cv::Mat &disparityMap, cv::Mat &rectifiedLeft,
                                                     cv::Mat &rectifiedRight, cv::Mat &Q) {
    cv::Mat points3D;

    cv::reprojectImageTo3D(disparityMap, points3D, Q);

    std::vector<cv::Point3d> points3DVector;
    for (int i = 0; i < points3D.rows; i++) {
        for (int j = 0; j < points3D.cols; j++) {
            cv::Vec3f point = points3D.at<cv::Vec3f>(i, j);
            if (cv::norm(point) < 1000) {
                points3DVector.push_back(cv::Point3d(point[0], point[1], point[2]));
            }
        }
    }

    return points3DVector;
}

static void estimate_pose(const std::vector<cv::Point2f> &pointsLeft,
                          const std::vector<cv::Point2f> &pointsRight, const cv::Mat &K_left,
                          cv::Mat &R, cv::Mat &T) {
    if (pointsLeft.size() < 8) {
        R.release();
        T.release();
        return;
    }

    cv::Mat inlierMask;
    cv::Mat E =
        cv::findEssentialMat(pointsLeft, pointsRight, K_left, cv::RANSAC, 0.99, 1.0, inlierMask);

    if (E.size() != cv::Size(3, 3)) {
        std::cout << "Essential matrix has incorrect size." << std::endl;
        return;
    }

    int numInliers = cv::recoverPose(E, pointsLeft, pointsRight, K_left, R, T, inlierMask);

    double inlierRatio = static_cast<double>(numInliers) / pointsLeft.size();

    if (inlierRatio < 0.9) {
        R.release();
        T.release();
        return;
    }

    if (T.at<double>(0, 0) < 0) {
        T = -T;
    }

    T.convertTo(T, CV_32F);
    R.convertTo(R, CV_32F);
}

static void display_matches(cv::Mat rectifiedLeft, std::vector<cv::KeyPoint> keypoints1,
                            cv::Mat rectifiedRight, std::vector<cv::KeyPoint> keypoints2,
                            std::vector<cv::DMatch> inlierMatches, bool block) {
    cv::Mat imgMatches;
    cv::drawMatches(rectifiedLeft, keypoints1, rectifiedRight, keypoints2, inlierMatches,
                    imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
                    cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("Matches", imgMatches);
    if (block)
        cv::waitKey(0);
    else
        cv::waitKey(1);
}

static void filter_matched_keypoints(std::vector<cv::KeyPoint> &keypoints_l,
                                     std::vector<cv::KeyPoint> &keypoints_r,
                                     std::vector<cv::DMatch> &matches,
                                     std::vector<cv::Point2f> &points_l,
                                     std::vector<cv::Point2f> &points_r) {
    points_l.clear();
    points_r.clear();

    for (const auto &match : matches) {
        // Use the queryIdx for the left image and trainIdx for the right image
        points_l.push_back(keypoints_l[match.queryIdx].pt);
        points_r.push_back(keypoints_r[match.trainIdx].pt);
    }
}

void stereo_process_images(cv::Mat &imgLeft, cv::Mat &imgRight, cv::Mat &R_estimated,
                           cv::Mat &T_estimated, std::vector<cv::Point3d> &points3D, bool block,
                           bool triangulate, bool estimate_displacement) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q, P_left_rect, P_right_rect;
    rectify_images(imgLeft, imgRight, rectifiedLeft, rectifiedRight, Q, P_left_rect, P_right_rect);

    cv::Mat K_left_rect = P_left_rect(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat K_right_rect = P_right_rect(cv::Rect(0, 0, 3, 3)).clone();

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    extract_features(rectifiedLeft, rectifiedRight, keypoints1, keypoints2, descriptors1,
                     descriptors2);

    std::vector<cv::DMatch> matches;
    match_descriptors(descriptors1, descriptors2, matches);

    std::vector<cv::Point2f> pointsLeft, pointsRight;
    filter_matched_keypoints(keypoints1, keypoints2, matches, pointsLeft, pointsRight);

    std::vector<cv::Point2f> transformedPoints;
    std::vector<cv::DMatch> inlierMatches;
    filter_correspondences(pointsLeft, pointsRight, matches, transformedPoints, inlierMatches);

    filter_matched_keypoints(keypoints1, keypoints2, inlierMatches, pointsLeft, pointsRight);

    display_matches(rectifiedLeft, keypoints1, rectifiedRight, keypoints2, inlierMatches, block);

    if (estimate_displacement)
        estimate_pose(pointsLeft, pointsRight, K_left_rect, R_estimated, T_estimated);

    if (triangulate)
        triangulate_points(P_left_rect, P_right_rect, pointsLeft, pointsRight, points3D);
}

void monocular_process_images(cv::Mat &img1, cv::Mat &img2, cv::Mat &R_estimated,
                              cv::Mat &T_estimated, bool block) {
    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T,
                              R);

    cv::Mat undistorted_img1, undistorted_img2;
    undistort_image(img1, K_left, D_left, undistorted_img1);
    undistort_image(img2, K_right, D_right, undistorted_img2);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    extract_features(undistorted_img1, undistorted_img2, keypoints1, keypoints2, descriptors1,
                     descriptors2);

    std::vector<cv::DMatch> matches;
    match_descriptors(descriptors1, descriptors2, matches);

    std::vector<cv::Point2f> pointsLeft, pointsRight;
    filter_matched_keypoints(keypoints1, keypoints2, matches, pointsLeft, pointsRight);

    std::vector<cv::Point2f> transformedPoints;
    std::vector<cv::DMatch> inlierMatches;
    filter_correspondences(pointsLeft, pointsRight, matches, transformedPoints, inlierMatches);

    filter_matched_keypoints(keypoints1, keypoints2, inlierMatches, pointsLeft, pointsRight);

    display_matches(undistorted_img1, keypoints1, undistorted_img2, keypoints2, inlierMatches,
                    block);

    estimate_pose(pointsLeft, pointsRight, K_left, R_estimated, T_estimated);
}

static void display_disparity_map(cv::Mat disparityMap, bool block) {
    cv::imshow("Disparity Map", disparityMap);
    if (block)
        cv::waitKey(0);
    else
        cv::waitKey(1);
}

void stereo_process_images_dense(cv::Mat &imgLeft, cv::Mat &imgRight,
                                 std::vector<cv::Point3d> &densePoints3D, bool block) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q, P_left_rect, P_right_rect;
    rectify_images(imgLeft, imgRight, rectifiedLeft, rectifiedRight, Q, P_left_rect, P_right_rect);

    cv::Mat disparityMap;
    compute_disparity_map(rectifiedLeft, rectifiedRight, disparityMap);

    display_disparity_map(disparityMap, block);

    densePoints3D = dense_reconstruction(disparityMap, rectifiedLeft, rectifiedRight, Q);
}
