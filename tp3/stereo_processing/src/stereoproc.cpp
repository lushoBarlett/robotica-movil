#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <vector>
#include <iostream>
#include "stereoproc.hpp"
#include "cam_params.hpp"

static void rectify_images(const cv::Mat imgLeft, const cv::Mat imgRight, cv::Mat* rectifiedLeft, cv::Mat* rectifiedRight, cv::Mat* Q, cv::Mat* P_left_rect, cv::Mat* P_right_rect) {
    if (imgLeft.empty() || imgRight.empty()) {
        std::cout << "Error: Could not load images." << std::endl;
        return;
    }

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);
   
    // Output matrices for rectification
    cv::Mat R1, R2;
    cv::Size imageSize = imgLeft.size();
    cv::stereoRectify(K_left, D_left, 
                      K_right, D_right, 
                      imageSize, R, T, 
                      R1, R2, *P_left_rect, *P_right_rect, *Q);

    // Rectification maps
    cv::Mat map1Left, map2Left, map1Right, map2Right;

    // Compute rectification maps for both images
    cv::initUndistortRectifyMap(K_left, D_left, R1, *P_left_rect, imageSize, CV_16SC2, map1Left, map2Left);
    cv::initUndistortRectifyMap(K_right, D_right, R2, *P_right_rect, imageSize, CV_16SC2, map1Right, map2Right);

    // Apply rectification
    cv::remap(imgLeft, *rectifiedLeft, map1Left, map2Left, cv::INTER_LINEAR);
    cv::remap(imgRight, *rectifiedRight, map1Right, map2Right, cv::INTER_LINEAR);
}

static void detect_keypoints(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint>* keypoint1, std::vector<cv::KeyPoint>* keypoint2) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    // Compute descriptors
    orb->detect(imgLeft, *keypoint1);
    orb->detect(imgRight, *keypoint2);
}

static void compute_descriptors(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint> keypoint1, std::vector<cv::KeyPoint> keypoint2, cv::Mat* descriptors1, cv::Mat* descriptors2) {
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    // Compute descriptors
    orb->compute(imgLeft, keypoint1, *descriptors1);
    orb->compute(imgRight, keypoint2, *descriptors2);
}

static void match_descriptors(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch>* matches) {
    cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true);  // NORM_HAMMING for binary descriptors like ORB and BRIEF

    // Match descriptors
    bfMatcher.match(descriptors1, descriptors2, *matches);

    // Filter matches by distance threshold
    float distanceThreshold = 30;
    std::vector<cv::DMatch> goodMatches;

    for (const auto& match : *matches) {
        if (match.distance < distanceThreshold) {
            goodMatches.push_back(match);
        }
    }
    *matches = goodMatches;
}

static void triangulate(cv::Mat P_left, cv::Mat P_right, std::vector<cv::Point2f> pointsLeft, std::vector<cv::Point2f> pointsRight, std::vector<cv::Point3d>* points3D) {    
    cv::Mat points4D;
    cv::triangulatePoints(P_left, P_right, pointsLeft, pointsRight, points4D);

    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat x = points4D.col(i);
        x /= x.at<float>(3);

        if (cv::norm(x) > 1000 || x.at<float>(2) <= 0)
            continue;
        
        points3D->push_back(cv::Point3d(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
    }
}

static void filter_correspondences(std::vector<cv::Point2f>& pointsLeft,
                                   std::vector<cv::Point2f>& pointsRight,
                                   std::vector<cv::DMatch>* matches,
                                   std::vector<cv::Point2f>* transformedPoints,
                                   std::vector<cv::DMatch>* inlierMatches) {
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
            if (inlierMatches)
                inlierMatches->push_back((*matches)[i]);
        }
    }

    // Update the input vectors with inliers
    pointsLeft = std::move(filteredPointsLeft);
    pointsRight = std::move(filteredPointsRight);

    // Transform points from left image to right image using the homography
    if (transformedPoints)
        cv::perspectiveTransform(pointsLeft, *transformedPoints, H);
}

static void compute_disparity_map(cv::Mat rectifiedLeft, cv::Mat rectifiedRight, cv::Mat* disparityMap) {    
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

    cv::normalize(disparityMapNotNormalized, *disparityMap, 0, 255, cv::NORM_MINMAX, CV_8U);
}

static std::vector<cv::Point3d> dense_reconstruction(cv::Mat& disparityMap, cv::Mat rectifiedLeft, cv::Mat rectifiedRight, cv::Mat Q) {
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

static void estimate_pose(
    const std::vector<cv::Point2f>& pointsLeft,
    const std::vector<cv::Point2f>& pointsRight,
    const cv::Mat& K_left, const cv::Mat& K_right,
    const cv::Mat& D_left, const cv::Mat& D_right,
    cv::Mat* R, cv::Mat* T
) {
    cv::Mat E = cv::findEssentialMat(pointsLeft, pointsRight, K_left, cv::RANSAC);

    if (E.size() != cv::Size(3, 3)) {
        std::cout << "Essential matrix has incorrect size." << std::endl;
        return;
    }

    cv::recoverPose(E, pointsLeft, pointsRight, K_left, *R, *T);

    T->convertTo(*T, CV_32F);
    R->convertTo(*R, CV_32F);
}

void process_images(cv::Mat imgLeft, cv::Mat imgRight, cv::Mat* R_estimated, cv::Mat* T_estimated, std::vector<cv::Point3d>* points3D, bool block) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q, P_left_rect, P_right_rect;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q, &P_left_rect, &P_right_rect);

    cv::Mat K_left_rect = P_left_rect(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat K_right_rect = P_right_rect(cv::Rect(0, 0, 3, 3)).clone();

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    detect_keypoints(rectifiedLeft, rectifiedRight, &keypoints1, &keypoints2);

    cv::Mat descriptors1;
    cv::Mat descriptors2;
    compute_descriptors(rectifiedLeft, rectifiedRight, keypoints1, keypoints2, &descriptors1, &descriptors2);

    std::vector<cv::DMatch> matches;
    match_descriptors(descriptors1, descriptors2, &matches);

    // Filter corresponding keypoints from good matches
    std::vector<cv::Point2f> pointsLeft, pointsRight;

    for (const auto& match : matches) {
        // Use the queryIdx for the left image and trainIdx for the right image
        pointsLeft.push_back(keypoints1[match.queryIdx].pt);
        pointsRight.push_back(keypoints2[match.trainIdx].pt);
    }

    std::vector<cv::Point2f> transformedPoints;
    std::vector<cv::DMatch> inlierMatches;
    filter_correspondences(pointsLeft, pointsRight, &matches, &transformedPoints, &inlierMatches);

    std::vector<cv::Point2f> inlierPointsLeft, inlierPointsRight;

    for (const auto& match : inlierMatches) {
        // Use queryIdx for the left image and trainIdx for the right image
        inlierPointsLeft.push_back(pointsLeft[match.queryIdx]);
        inlierPointsRight.push_back(pointsRight[match.trainIdx]);
    }

    cv::Mat imgMatches;
    cv::drawMatches(rectifiedLeft, keypoints1, rectifiedRight, keypoints2, inlierMatches, imgMatches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("Matches", imgMatches);
    if (block)
        cv::waitKey(0);
    else
        cv::waitKey(1);

    double norm_diff = cv::norm(K_left_rect, K_right_rect, cv::NORM_L2);
    double tolerance = 1e-6;
    if (norm_diff > tolerance) {
        std::cerr << "Matrices are not equal!" << std::endl;
        std::cerr << "K_left_rect: " << std::endl << K_left_rect << std::endl;
        std::cerr << "K_right_rect: " << std::endl << K_right_rect << std::endl;
        throw std::invalid_argument("Matrices are not equal");
    }

    if (inlierPointsLeft.size() < 8) {
        R_estimated->release();
        T_estimated->release();
    } else {
        estimate_pose(inlierPointsLeft, inlierPointsRight, K_left_rect, K_right_rect, D_left, D_right, R_estimated, T_estimated);
    }

    triangulate(P_left_rect, P_right_rect, inlierPointsLeft, inlierPointsRight, points3D);
}

void process_images_dense(cv::Mat imgLeft, cv::Mat imgRight, std::vector<cv::Point3d>* densePoints3D, bool block) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q, P_left_rect, P_right_rect;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q, &P_left_rect, &P_right_rect);

    cv::Mat disparityMap;
    compute_disparity_map(rectifiedLeft, rectifiedRight, &disparityMap);

    cv::imshow("Disparity Map", disparityMap);
    
    if (block)
        cv::waitKey(0);
    else
        cv::waitKey(1);

    *densePoints3D = dense_reconstruction(disparityMap, rectifiedLeft, rectifiedRight, Q);
}
