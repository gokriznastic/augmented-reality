/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Functions for various steps used during the calibration of camera and AR system.
*/

#ifndef calibrate_hpp
#define calibrate_hpp

#include <stdio.h>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 the function detects the corners present in the checkerboard grid and draws them.
 This function also populates given vector with image pixel coordinates of corners detected.
 */
bool extractCorners(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &corners, bool drawCorners);

/*
 Given a vector of points having image pixel coordinates of detected corners,
 this function populates the vector of points in world coordinates for the checkerboard target.
 It also populates the vectors for storing multiple point sets and corner sets to be used in calibration.
 */
int selectCalibration(std::vector<cv::Point2f> &corners, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list);

/*
 Given vectors having a list of point sets and corner sets, an initial camera matrix,
 this function generates the calibration and calculates the calibrated camera matrix and distortion coeffecients.
 This function also returns the reprojection error after performing calibration.
 */
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &corners_list, cv::Mat &camera_matrix, cv::Mat &dist_coeff);

/*
 Given camera matrix and distance coefficients,
 this function saves the current calibration into a CSV file to be retrieved later for calculating camera pose.
 */
int saveCalibration(cv::Mat &camera_matrix, cv::Mat &dist_coeff);

#endif /* calibrate_hpp */
