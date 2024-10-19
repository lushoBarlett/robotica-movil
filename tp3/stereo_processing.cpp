#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/viz.hpp>
#include <fstream>
#include <vector>

// Function to set up stereo camera matrices
void setupStereoCameraMatrices(cv::Mat& D_left, cv::Mat& K_left, cv::Mat& R_left, cv::Mat& P_left,
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

void rectify_images(const cv::Mat imgLeft, const cv::Mat imgRight, cv::Mat* rectifiedLeft, cv::Mat* rectifiedRight, cv::Mat* Q) {
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

void detect_keypoints(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint>* keypoint1, std::vector<cv::KeyPoint>* keypoint2) {
    int threshold = 30;
    bool nonmaxSuppression = true;
    cv::Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(threshold, nonmaxSuppression);

    fastDetector->detect(imgLeft, *keypoint1);
    fastDetector->detect(imgRight, *keypoint2);
}

void compute_descriptors(const cv::Mat imgLeft, const cv::Mat imgRight, std::vector<cv::KeyPoint> keypoint1, std::vector<cv::KeyPoint> keypoint2, cv::Mat* descriptors1, cv::Mat* descriptors2) {
    // Create an ORB keypoint detector and descriptor extractor
    int maxKeypoints = 1000; // Maximum number of keypoints to retain
    cv::Ptr<cv::ORB> orb = cv::ORB::create(maxKeypoints);

    // Compute descriptors
    orb->compute(imgLeft, keypoint1, *descriptors1);
    orb->compute(imgRight, keypoint2, *descriptors2);
}

void match_descriptors(cv::Mat descriptors1, cv::Mat descriptors2, std::vector<cv::DMatch>* matches) {
    cv::BFMatcher bfMatcher(cv::NORM_HAMMING, true);  // NORM_HAMMING for binary descriptors like ORB and BRIEF

    // Match descriptors
    bfMatcher.match(descriptors1, descriptors2, *matches);

    // Filter matches by distance threshold
    float distanceThreshold = 20;
    std::vector<cv::DMatch> goodMatches;

    for (const auto& match : *matches) {
        if (match.distance < distanceThreshold) {
            goodMatches.push_back(match);
        }
    }
    *matches = goodMatches;
}

void triangulate(cv::Mat P_left, cv::Mat P_right, std::vector<cv::Point2f> pointsLeft, std::vector<cv::Point2f> pointsRight, std::vector<cv::Point3d>* points3D) {
    cv::Mat points4D;
    cv::triangulatePoints(P_left, P_right, pointsLeft, pointsRight, points4D);

    for (int i = 0; i < points4D.cols; i++) {
        cv::Mat x = points4D.col(i);
        x /= x.at<float>(3);

        // if the point is too far away, ignore it
        if (cv::norm(x) > 1000)
            continue;

        points3D->push_back(cv::Point3d(x.at<float>(0), x.at<float>(1), x.at<float>(2)));
    }
}

void filter_correspondences(std::vector<cv::Point2f>& pointsLeft, std::vector<cv::Point2f>& pointsRight, std::vector<cv::Point3d>& points3D) {
    cv::Mat H = cv::findHomography(pointsLeft, pointsRight, cv::RANSAC);

    // Transform points from left image to right image
    std::vector<cv::Point2f> transformedPoints;
    cv::perspectiveTransform(pointsLeft, transformedPoints, H);

    // Display transformed points
    cv::Mat imgLeft = cv::imread("data/left-0000.png", cv::IMREAD_COLOR);
    cv::Mat imgRight = cv::imread("data/right-0000.png", cv::IMREAD_COLOR);

    for (const auto& point : transformedPoints) {
        cv::circle(imgRight, point, 3, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    cv::imshow("Transformed points", imgRight);
    cv::waitKey(0);
}

cv::Mat quat2rot(cv::Vec4d q) {
    double w = q[0];
    double x = q[1];
    double y = q[2];
    double z = q[3];

    cv::Mat R = (cv::Mat_<double>(3, 3) << 1 - 2 * y * y - 2 * z * z, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w,
                                           2 * x * y + 2 * z * w, 1 - 2 * x * x - 2 * z * z, 2 * y * z - 2 * x * w,
                                           2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x * x - 2 * y * y);

    return R;
}

void read_ground_truth_positions(std::vector<cv::Mat>& outposes) {
    std::string filename = "data/ground_truth.csv";
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cout << "Error: Could not open file." << std::endl;
        return;
    }

    std::vector<cv::Point3d> outpos;
    std::vector<cv::Mat> outrot;

    std::string line;
    for (int i = 0; std::getline(file, line); i++) {
        if (i == 0)
            continue;

        std::stringstream ss(line);
        std::string token;

        std::vector<double> values;
        while (std::getline(ss, token, ','))
            values.push_back(std::stod(token));

        outpos.push_back(cv::Point3d(values[1], values[2], values[3]));
        outrot.push_back(quat2rot(cv::Vec4d(values[4], values[5], values[6], values[7])));
    }

    file.close();

    for (int i = 0; i < outpos.size(); i++) {
        cv::Mat pose = cv::Mat::eye(4, 4, CV_64F);

        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                pose.at<double>(j, k) = outrot[i].at<double>(j, k);

        pose.at<double>(0, 3) = outpos[i].x;
        pose.at<double>(1, 3) = outpos[i].y;
        pose.at<double>(2, 3) = outpos[i].z;

        outposes.push_back(pose);
    }
}

void feature_mapping(const std::vector<cv::Point3d>& points3D) {
    cv::viz::Viz3d window("3D Plot");
    cv::viz::WCloud cloudWidget(points3D, cv::viz::Color::green());
    window.showWidget("Cloud", cloudWidget);
    window.spin();
}

void compute_disparity_map(cv::Mat& disparityMap) {
    cv::Mat imgLeft = cv::imread("data/left-0000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat imgRight = cv::imread("data/right-0000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

    cv::Mat disparityMapNotNormalized;
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(16, 9);
    stereo->compute(rectifiedLeft, rectifiedRight, disparityMapNotNormalized);

    cv::normalize(disparityMapNotNormalized, disparityMap, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::imshow("Disparity Map", disparityMap);
    cv::waitKey(0);
}

void dense_reconstruction(cv::Mat& disparityMap, cv::Mat& points3D) {
    cv::Mat imgLeft = cv::imread("data/left-0000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat imgRight = cv::imread("data/right-0000.png", cv::IMREAD_GRAYSCALE);
    cv::Mat rectifiedLeft, rectifiedRight;

    cv::Mat Q;
    rectify_images(imgLeft, imgRight, &rectifiedLeft, &rectifiedRight, &Q);

    cv::reprojectImageTo3D(disparityMap, points3D, Q);

    cv::viz::Viz3d window("3D Plot");
    std::vector<cv::Point3d> points3DVector;
    for (int i = 0; i < points3D.rows; i++) {
        for (int j = 0; j < points3D.cols; j++) {
            cv::Point3d point = points3D.at<cv::Point3d>(i, j);
            if (cv::norm(point) < 1000) {
                points3DVector.push_back(point);
            }
        }
    }
    cv::viz::WCloud cloudWidget(points3DVector, cv::viz::Color::green());
    window.showWidget("Cloud", cloudWidget);
    window.spin();
}

void dense_mapping() {
    // TODO
}

void estimate_pose(
    const std::vector<cv::Point2f>& pointsLeft,
    const std::vector<cv::Point2f>& pointsRight,
    const cv::Mat& K_left, const cv::Mat& K_right,
    const cv::Mat& D_left, const cv::Mat& D_right
) {
    std::vector<cv::Point2f> pointsLeftUndistorted, pointsRightUndistorted;

    cv::undistortPoints(pointsLeft, pointsLeftUndistorted, K_left, D_left);
    cv::undistortPoints(pointsRight, pointsRightUndistorted, K_right, D_right);

    cv::Mat E = cv::findEssentialMat(pointsLeftUndistorted, pointsRightUndistorted, cv::Mat::eye(3, 3, CV_64F), cv::RANSAC);

    cv::Mat R, T;

    cv::recoverPose(E, pointsLeftUndistorted, pointsRightUndistorted, cv::Mat::eye(3, 3, CV_64F), R, T);

    std::cout << "Rotation matrix: " << R << std::endl;
    std::cout << "Translation vector: " << T << std::endl;
}

void to_homogenous_coords(const std::vector<cv::Point3d>& points3D, std::vector<cv::Mat>& points3DHomogenous) {
    for (const auto& point : points3D) {
        cv::Mat pointHomogenous = (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1);
        points3DHomogenous.push_back(pointHomogenous);
    }
}

void from_homogenous_coords(const std::vector<cv::Mat>& points3DHomogenous, std::vector<cv::Point3d>& points3D) {
    for (const auto& point : points3DHomogenous) {
        cv::Point3d point3D;
        double w = point.at<double>(3, 0);
        point3D.x = point.at<double>(0, 0) / w;
        point3D.y = point.at<double>(1, 0) / w;
        point3D.z = point.at<double>(2, 0) / w;
        points3D.push_back(point3D);
    }
}

int main() {
    cv::Mat imgLeft = cv::imread("data/left-0000.png", cv::COLOR_BGR2GRAY);
    cv::Mat imgRight = cv::imread("data/right-0000.png", cv::COLOR_BGR2GRAY);
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

    filter_correspondences(pointsLeft, pointsRight, points3D);

    std::vector<cv::Mat> poses;
    read_ground_truth_positions(poses);

    std::vector<cv::Mat> points3DHomogenous;
    to_homogenous_coords(points3D, points3DHomogenous);

    for (auto& pointH : points3DHomogenous)
        pointH = poses[0] * pointH;

    std::vector<cv::Point3d> points3DMoved;
    from_homogenous_coords(points3DHomogenous, points3DMoved);

    feature_mapping(points3D);
    feature_mapping(points3DMoved);

    cv::Mat disparityMap;
    compute_disparity_map(disparityMap);

    cv::Mat densePoints3D;
    dense_reconstruction(disparityMap, densePoints3D);

    // dense_mapping();

    estimate_pose(pointsLeft, pointsRight, K_left, K_right, D_left, D_right);
}
