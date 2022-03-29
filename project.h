/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Functions for projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#ifndef project_hpp
#define project_hpp

#include <stdio.h>
#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

/*
 Given the CSV with calibratiion stats,
 this function retrieves the calibrated camera matrix and distortion coefficients.
 */
int readCalibration(std::string csv_filename, cv::Mat &camera_matrix, cv::Mat &dist_coeff);

/*
 Given vector containing current point set and corner set, calibrated camera matrix and distortion coeffcients,
 this function estimnates the position of the camera relative to the target and populates arrays with rotation and translation data.
 */
int calcCameraPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &corners, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans);

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
 Given a cv::Mat of the image frame and a cv::Mat of the output frame,
 this function detects the corners using Harris corners detection method and draw them on output frame.
 */
int detectHarrisCorners(cv::Mat &src, cv::Mat &dst);

#endif /* project_hpp */
