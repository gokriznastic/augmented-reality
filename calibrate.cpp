/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Function implementations for various steps used during the calibration of camera and AR system.
*/

#include "calibrate.h"
#include "csv_util.h"

/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 the function detects the corners present in the checkerboard grid and draws them.
 This function also populates given vector with image pixel coordinates of corners detected.
 */
bool extractCorners(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &corners, bool drawCorners)
{
    dst = src.clone();
       
    bool found = cv::findChessboardCorners(src, cv::Size(9, 6), corners);
    
    //std::cout << "No. of corners detected:- " << corners.size() << std::endl;
    //std::cout << "Co-ordinate of top left corner:- " << corners[0].x << " " << corners[0].y << std::endl;
    
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    
    if (found == true) {
        cv::cornerSubPix(gray, corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,30,0.1));
    }
    
    if (drawCorners) {
        cv::drawChessboardCorners(dst, cv::Size(6, 9), corners, found);
    }
    
    return(found);
}


/*
 Given a vector of points having image pixel coordinates of detected corners,
 this function populates the vector of points in world coordinates for the checkerboard target.
 It also populates the vectors for storing multiple point sets and corner sets to be used in calibration.
 */
int selectCalibration(std::vector<cv::Point2f> &corners, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list)
{
    int cols = 9;
    
    for(int k=0; k<corners.size(); k++)
    {
        float i = (float)(k % cols);
        float j = (float)(-1 * k/cols);
        cv::Vec3f point(i, j, 0);
        points.push_back(point);
    }

    corners_list.push_back(corners);
    points_list.push_back(points);
    
    return(0);
}


/*
 Given vectors having a list of point sets and corner sets, an initial camera matrix,
 this function generates the calibration and calculates the calibrated camera matrix and distortion coeffecients.
 */
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &corners_list, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    std::vector<cv::Mat> rot, trans;
    
    float error = cv::calibrateCamera(points_list,
                                corners_list,
                                cv::Size(1280, 720),
                                camera_matrix,
                                dist_coeff,
                                rot,
                                trans,
                                cv::CALIB_FIX_ASPECT_RATIO,
                                cv::TermCriteria(cv::TermCriteria::MAX_ITER+cv::TermCriteria::EPS, 30, DBL_EPSILON));
    
    return(error);
}


/*
 Given camera matrix and distance coeffcients,
 this function saves the current calibration into a csv file to be retrieved later for calculating camera pose.
 */
int saveCalibration(cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    std::string fileName = "intrinsic_data.csv";
    char* fileName_char = new char[fileName.length() + 1];
    strcpy(fileName_char, fileName.c_str());

    std::string columnName = "camera_matrix";
    char* labelName_char = new char[columnName.length() + 1];
    strcpy(labelName_char, columnName.c_str());

    std::vector<float> camVector;
    for (int i = 0; i < camera_matrix.rows; i++) {
        for (int j = 0; j < camera_matrix.cols; j++) {
            float f_val = (float) camera_matrix.at<double>(i, j);
            camVector.push_back(f_val);
        }
    }
    append_image_data_csv(fileName_char, labelName_char, camVector);

    columnName = "distortion_coeff";
    char* label_char = new char[columnName.length() + 1];
    strcpy(label_char, columnName.c_str());

    std::vector<float> distVector;
    for (int i = 0; i < dist_coeff.rows; i++) {
        for (int j = 0; j < dist_coeff.cols; j++) {
            float f_val = (float) dist_coeff.at<double>(i, j);
            distVector.push_back(f_val);
        }
    }
    append_image_data_csv(fileName_char, label_char, distVector);
    
    return(0);
}
