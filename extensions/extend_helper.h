/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Functions for various steps used during the calibration of camera and projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#ifndef calibrate_hpp
#define calibrate_hpp

#include <stdio.h>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 the function detects the circles centers present in the circle grid and draws them.
 This function also populates given vector with image pixel coordinates of centers detected.
 */
bool extractCircleCenters(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &centers, bool drawCenters);

/*
 Given a vector of points having image pixel coordinates of detected circle centers,
 this function populates the vector of points in world coordinates for the checkerboard target.
 It also populates the vectors for storing multiple point sets and center sets to be used in calibration.
 */
int selectCalibration(std::vector<cv::Point2f> &centers, std::vector<std::vector<cv::Point2f>> &centers_list, std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list);

/*
 Given vectors having a list of point sets and center sets, an initial camera matrix,
 this function generates the calibration and calculates the calibrated camera matrix and distortion coeffecients.
 This function also returns the reprojection error after performing calibration.
 */
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &centers_list, cv::Mat &camera_matrix, cv::Mat &dist_coeff);

/*
 Given camera matrix and distance coefficients,
 this function saves the current calibration into a CSV file to be retrieved later for calculating camera pose.
 */
int saveCalibration(cv::Mat &camera_matrix, cv::Mat &dist_coeff);

/*
 Given the CSV with calibratiion stats,
 this function retrieves the calibrated camera matrix and distortion coefficients.
 */
int readCalibration(std::string csv_filename, cv::Mat &camera_matrix, cv::Mat &dist_coeff);

/*
 Given vector containing current point set and center set, calibrated camera matrix and distortion coeffcients,
 this function estimnates the position of the camera relative to the target and populates arrays with rotation and translation data.
 */
int calcCameraPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &centers, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans);

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world coordinates of axes to image pixel
 coordinates on the image frame and draws lines between these points to generate the 3D axes at origin.
 */
int draw3dAxes(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans);

/*
 Given a cv::Mat of the image frame, calibrated camera matrix, distortion coefficients, rotation and translation data
 from the current estimated camera position, this function projects 3D world vertices of virtual shapes to image pixel
 coordinates on the image frame and draws lines between them to generate 3D virtual objects on the target.
 */
int draw3dObject(cv::Mat &src, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans);

/*
 Given a cv::Mat of the image frame, a cv::Mat of the output frame, calibrated camera matrix, distortion coefficients, rotation & translation data and filename for artwork image,
 this function reads the artwork image and draws artwork image on the target using perspective transformation.
 */
int drawOnTarget(cv::Mat &src, cv::Mat &dst, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans, std::string img_filename);

#endif /* calibrate_hpp */
