#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/viz.hpp>
#include <fstream>
#include <vector>
#include "stereoproc.hpp"

// Function to set up stereo camera matrices
static void setupStereoCameraMatrices(cv::Mat& D_left, cv::Mat& K_left, cv::Mat& R_left, cv::Mat& P_left,
                               cv::Mat& D_right, cv::Mat& K_right, cv::Mat& R_right, cv::Mat& P_right,
                               cv::Mat& T, cv::Mat& R) {
    // Left camera parameters
    D_left = (cv::Mat_<double>(1, 5) << -0.28528567852276154, 0.0755173390138138, 0.00042325617975809654, 6.82046332766036e-05, 0.0);
    K_left = (cv::Mat_<double>(3, 3) << 461.4083655730878, 0.0, 364.74303421116605,
                                        0.0, 460.1948345348428, 247.5842300899086,
                                        0.0, 0.0, 1.0);
    R_left = (cv::Mat_<double>(3, 3) << 0.9999291388891489, -0.0007704353780885541, -0.011879546697297074,
                                        0.0008675722844585223, 0.9999662172978196, 0.008173835172110863,
                                        0.011872847962317707, -0.008183562330537164, 0.9998960269892295);
    P_left = (cv::Mat_<double>(3, 4) << 439.40304644813335, 0.0, 383.1048240661621, 0.0,
                                        0.0, 439.40304644813335, 255.35926055908203, 0.0,
                                        0.0, 0.0, 1.0, 0.0);

    // Right camera parameters
    D_right = (cv::Mat_<double>(1, 5) << -0.2852945932144836, 0.07556094233309328, -0.00016457104721592913, 4.67978311690005e-05, 0.0);
    K_right = (cv::Mat_<double>(3, 3) << 461.22152512371986, 0.0, 373.630015164131,
                                         0.0, 460.3701926236939, 253.4813239671563,
                                         0.0, 0.0, 1.0);
    R_right = (cv::Mat_<double>(3, 3) << 0.999777876762165, -0.003390543364174486, -0.020801474770623506,
                                         0.003220288489111239, 0.9999610891135602, -0.00821279494868492,
                                         0.020828511204215524, 0.008143983946318862, 0.999749893046505);
    P_right = (cv::Mat_<double>(3, 4) << 439.40304644813335, 0.0, 383.1048240661621, -48.169298967129315,
                                         0.0, 439.40304644813335, 255.35926055908203, 0.0,
                                         0.0, 0.0, 1.0, 0.0);

    // Rotation and translation between the cameras
    T = (cv::Mat_<double>(3, 1) << -0.10960005816018638, 0.000371686310074846, 0.0022803493632622346);
    R = (cv::Mat_<double>(3, 3) << 0.9999571189710634, 0.0022794640328858873, 0.008975759734597452,
                                   -0.0024260722968527915, 0.9998632731202098, 0.016356932430961323,
                                   -0.008937247467810473, -0.01637800687090375, 0.9998259280988044);
}

static void rectify_images(const cv::Mat imgLeft, const cv::Mat imgRight, cv::Mat* rectifiedLeft, cv::Mat* rectifiedRight, cv::Mat* Q) {
    if (imgLeft.empty() || imgRight.empty()) {
        std::cout << "Error: Could not load images." << std::endl;
        return;
    }

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);
   
    // Output matrices for rectification
    cv::Mat R1, R2, P1, P2;
    cv::Size imageSize = imgLeft.size();
    cv::stereoRectify(K_left, D_left, 
                      K_right, D_right, 
                      imageSize, R, T, 
                      R1, R2, P1, P2, *Q);

    // Rectification maps
    cv::Mat map1Left, map2Left, map1Right, map2Right;

    // Compute rectification maps for both images
    cv::initUndistortRectifyMap(K_left, D_left, R1, P1, imageSize, CV_16SC2, map1Left, map2Left);
    cv::initUndistortRectifyMap(K_right, D_right, R2, P2, imageSize, CV_16SC2, map1Right, map2Right);

    // Apply rectification
    cv::remap(imgLeft, *rectifiedLeft, map1Left, map2Left, cv::INTER_LINEAR);
    cv::remap(imgRight, *rectifiedRight, map1Right, map2Right, cv::INTER_LINEAR);
}

static void detect_keypoints(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint>* keypoint1, std::vector<cv::KeyPoint>* keypoint2) {
    int threshold = 30;
    bool nonmaxSuppression = true;
    cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(threshold, nonmaxSuppression);

    fastDetector->detect(imgLeft, *keypoint1);
    fastDetector->detect(imgRight, *keypoint2);
}

static void compute_descriptors(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint> keypoint1, std::vector<cv::KeyPoint> keypoint2, cv::Mat* descriptors1, cv::Mat* descriptors2) {
    // Create an ORB keypoint detector and descriptor extractor
    int maxKeypoints = 1000; // Maximum number of keypoints to retain
    cv::Ptr<cv::ORB> orb = cv::ORB::create(maxKeypoints);

    // Compute descriptors
    orb->compute(imgLeft, keypoint1, *descriptors1);
    orb->compute(imgRight, keypoint2, *descriptors2);
}

static void match_descriptors(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch>* matches) {
    cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true);  // NORM_HAMMING for binary descriptors like ORB and BRIEF

    // Match descriptors
    bfMatcher.match(descriptors1, descriptors2, *matches);

    // Filter matches by distance threshold
    float distanceThreshold = 10;
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
        
        if (cv::norm(x) > 1000)
            continue;
        
        points3D->push_back(cv::Point3d(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
    }
}

static void filter_correspondences(std::vector<cv::Point2f>& pointsLeft, std::vector<cv::Point2f>& pointsRight, std::vector<cv::Point3d>& points3D, std::vector<cv::Point2f>* transformedPoints) {
    cv::Mat H = cv::findHomography(pointsLeft, pointsRight, cv::RANSAC);

    // Transform points from left image to right image
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
    std::vector<cv::Point2f> pointsLeftUndistorted, pointsRightUndistorted;

    cv::undistortPoints(pointsLeft, pointsLeftUndistorted, K_left, D_left);
    cv::undistortPoints(pointsRight, pointsRightUndistorted, K_right, D_right);

    cv::Mat E = cv::findEssentialMat(pointsLeftUndistorted, pointsRightUndistorted, cv::Mat::eye(3, 3, CV_64F), cv::RANSAC);

    cv::recoverPose(E, pointsLeftUndistorted, pointsRightUndistorted, cv::Mat::eye(3, 3, CV_64F), *R, *T);
}

void rectifiyng_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight, Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);
    cv::imshow("Rectified image left", rectifiedLeft);
    cv::imshow("Rectified image right", rectifiedRight);
    cv::waitKey(0);
}

void keypoints_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;
    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;
    std::vector<cv::KeyPoint> keypoints1, keypoints2;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

    detect_keypoints(rectifiedLeft, rectifiedRight, &keypoints1, &keypoints2);

    cv::Mat keypointImageLeft;
    cv::drawKeypoints(rectifiedLeft, keypoints1, keypointImageLeft, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::Mat keypointImageRight;
    cv::drawKeypoints(rectifiedRight, keypoints2, keypointImageRight, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("Keypoints left", keypointImageLeft);
    cv::imshow("Keypoints right", keypointImageRight);
    cv::waitKey(0);
}

void matching_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    detect_keypoints(rectifiedLeft, rectifiedRight, &keypoints1, &keypoints2);

    cv::Mat descriptors1;
    cv::Mat descriptors2;
    compute_descriptors(rectifiedLeft, rectifiedRight, keypoints1, keypoints2, &descriptors1, &descriptors2);

    std::vector<cv::DMatch> matches;
    match_descriptors(descriptors1, descriptors2, &matches);

    cv::Mat imgMatches;
    cv::drawMatches(rectifiedLeft, keypoints1, rectifiedRight, keypoints2, matches, imgMatches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::imshow("Matches", imgMatches);
    cv::waitKey(0);
}


void homography_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

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

    std::vector<cv::Point3d> points3D;
    triangulate(P_left, P_right, pointsLeft, pointsRight, &points3D);

    std::vector<cv::Point2f> transformedPoints;
    filter_correspondences(pointsLeft, pointsRight, points3D, &transformedPoints);

    for (const auto& point : transformedPoints) {
        cv::circle(rectifiedRight, point, 3, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    cv::imshow("Transformed points", rectifiedRight);
    cv::waitKey(0);
}

void disparity_map_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

    cv::Mat disparityMap;
    compute_disparity_map(rectifiedLeft, rectifiedRight, &disparityMap);

    cv::imshow("Disparity Map", disparityMap);
    cv::waitKey(0);
}



void pose_estimation_process(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

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

    cv::Mat R_estimated, T_estimated;
    estimate_pose(pointsLeft, pointsRight, K_left, K_right, D_left, D_right, &R_estimated, &T_estimated);

    std::cout << "Rotation matrix: " << R_estimated << std::endl;
    std::cout << "Translation vector: " << T_estimated << std::endl;
}

std::vector<cv::Point3d> dense_point_cloud(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

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

    cv::Mat disparityMap;
    compute_disparity_map(rectifiedLeft, rectifiedRight, &disparityMap);

    std::vector<cv::Point3d> densePoints3D = dense_reconstruction(disparityMap, rectifiedLeft, rectifiedRight, Q);

    return densePoints3D;
}

std::vector<cv::Point3d> triangulateKeyPoints(cv::Mat imgLeft, cv::Mat imgRight) {
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat D_left, K_left, R_left, P_left;
    cv::Mat D_right, K_right, R_right, P_right;
    cv::Mat T, R;

    setupStereoCameraMatrices(D_left, K_left, R_left, P_left, D_right, K_right, R_right, P_right, T, R);

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

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

    std::vector<cv::Point3d> points3D;
    triangulate(P_left, P_right, pointsLeft, pointsRight, &points3D);

    return points3D;
}