/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Functions for projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#include "project.h"
#include "csv_util.h"

/*
 Given the CSV with calibratiion stats,
 this function retrieves the calibrated camera matrix and distortion coefficients.
 */
int readCalibration(std::string csv_filename, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    char* fname_char = new char[csv_filename.length() + 1];
    strcpy(fname_char, csv_filename.c_str());
    
    std::cout << "Retrieving saved calibration..." << std::endl;
    std::vector<char*> featureName;
    std::vector<std::vector<float>> data;
    read_image_data_csv(fname_char, featureName, data);

    camera_matrix.at<double>(0, 0) = (double)data[0][0];
    camera_matrix.at<double>(0, 1) = (double)data[0][1];
    camera_matrix.at<double>(0, 2) = (double)data[0][2];
    camera_matrix.at<double>(1, 0) = (double)data[0][3];
    camera_matrix.at<double>(1, 1) = (double)data[0][4];
    camera_matrix.at<double>(1, 2) = (double)data[0][5];
    camera_matrix.at<double>(2, 0) = (double)data[0][6];
    camera_matrix.at<double>(2, 1) = (double)data[0][7];
    camera_matrix.at<double>(2, 2) = (double)data[0][8];
    
    dist_coeff = cv::Mat(1, 5, CV_32F);
    
    for (int i=0;i<5;i++) {
        dist_coeff.at<float>(0, i) = data[1][i];
    }

    return(0);
}


/*
 Given vector containing current point set and corner set, calibrated camera matrix and distortion coeffcients,
 this function estimnates the position of the camera relative to the target and populates arrays with rotation and translation data.
 */
int calcCameraPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &corners, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    cv::solvePnP(points, corners, camera_matrix, dist_coeff, rot, trans);
    
    return(0);
}


/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world coordinates of axes to image pixel
 coordinates on the image frame and draws lines between these points to generate the 3D axes at origin.
 */
int draw3dAxes(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    std::vector<cv::Vec3f> points;
    points.push_back(cv::Vec3f({0, 0, 0}));
    points.push_back(cv::Vec3f({2, 0, 0}));
    points.push_back(cv::Vec3f({0, -2, 0}));
    points.push_back(cv::Vec3f({0, 0, 2}));
    
    std::vector<cv::Point2f> corners;
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);
    
    cv::arrowedLine(src, corners[0], corners[1], cv::Scalar(0, 0, 255), 5);
    cv::arrowedLine(src, corners[0], corners[2], cv::Scalar(0, 255, 0), 5);
    cv::arrowedLine(src, corners[0], corners[3], cv::Scalar(255, 0, 0), 5);
    
    return(0);
}


/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world vertices of virtual shapes to image pixel
 coordinates on the image frame and draws lines between them to generate 3D virtual objects on the target.
 */
int draw3dObject(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    // PYRAMID
    std::vector<cv::Vec3f> points;
    points.push_back(cv::Vec3f({2, -2, 3})); // center
    points.push_back(cv::Vec3f({3, -1, 0})); // tr
    points.push_back(cv::Vec3f({3, -3, 0})); // br
    points.push_back(cv::Vec3f({1, -3, 0})); // bl
    points.push_back(cv::Vec3f({1, -1, 0})); // tl
    
    std::vector<cv::Point2f> corners;
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    cv::line(src, corners[0], corners[1], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[0], corners[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[0], corners[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[0], corners[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[1], corners[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[2], corners[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[3], corners[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, corners[4], corners[1], cv::Scalar(0, 255, 255), 5);
        
    points.clear();
    corners.clear();
    
    // CUBE
    points.push_back(cv::Vec3f({8, -5, 0})); // br
    points.push_back(cv::Vec3f({8, -5, 2}));
    points.push_back(cv::Vec3f({6, -5, 0})); // bl
    points.push_back(cv::Vec3f({6, -5, 2}));
    points.push_back(cv::Vec3f({8, -3, 0})); // tr
    points.push_back(cv::Vec3f({8, -3, 2}));
    points.push_back(cv::Vec3f({6, -3, 0})); // tl
    points.push_back(cv::Vec3f({6, -3, 2}));
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    cv::line(src, corners[0], corners[2], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[4], corners[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[0], corners[4], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[2], corners[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[1], corners[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[5], corners[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[1], corners[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[3], corners[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[0], corners[1], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[2], corners[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[4], corners[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, corners[6], corners[7], cv::Scalar(255, 255, 0), 5);
    
    points.clear();
    corners.clear();
    
    // PRISM
    points.push_back(cv::Vec3f({6, -1, 2})); // br
    points.push_back(cv::Vec3f({4, -1, 2})); // bl
    points.push_back(cv::Vec3f({6, 0, 2})); // tr
    points.push_back(cv::Vec3f({4, 0, 2})); // tl
    points.push_back(cv::Vec3f({6, -1, 4}));
    points.push_back(cv::Vec3f({6, 0, 4}));
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, corners);

    cv::line(src, corners[0], corners[1], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[1], corners[3], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[2], corners[3], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[0], corners[2], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[4], corners[5], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[0], corners[4], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[2], corners[5], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[1], corners[4], cv::Scalar(255, 0, 255), 5);
    cv::line(src, corners[3], corners[5], cv::Scalar(255, 0, 255), 5);

    return(0);
}


/*
 Given a cv::Mat of the image frame and a cv::Mat of the output frame,
 this function detects the corners using Harris corners detection method and draw them on output frame.
 */
int detectHarrisCorners(cv::Mat &src, cv::Mat &dst)
{
    int thresh = 150;
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    
    dst = src.clone();
    cv::Mat gray, tmp, tmp_norm;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    tmp = cv::Mat::zeros(src.size(), CV_32FC1);

    // Detecting corners
    cornerHarris(gray, tmp, blockSize, apertureSize, k);

    // Normalizing
    cv::normalize(tmp, tmp_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // Drawing a circle around corners
    for(int j = 0; j < tmp_norm.rows ; j++) {
        for(int i = 0; i < tmp_norm.cols; i++) {
            if((int) tmp_norm.at<float>(j,i) > thresh) {
                cv::circle(dst, cv::Point(i, j), 2, cv::Scalar(0, 0, 255), 2, 8, 0 );
            }
        }
    }
        
    return(0);
}
