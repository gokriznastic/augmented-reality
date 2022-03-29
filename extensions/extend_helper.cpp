/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

Function implementations for various steps used during the calibration of camera and projecting 3D points in world coordinates to 2D image pixel coordinates.
*/

#include "extend_helper.h"
#include "csv_util.h"

/*
 Given a cv::Mat of the image frame, cv::Mat for the output and vector of points
 the function detects the circles centers present in the circle grid and draws them.
 This function also populates given vector with image pixel coordinates of centers detected.
 */
bool extractCircleCenters(cv::Mat &src, cv::Mat &dst, std::vector<cv::Point2f> &centers, bool drawCenters)
{
    dst = src.clone();
       
    bool found = cv::findCirclesGrid(src, cv::Size(4, 11), centers, cv::CALIB_CB_ASYMMETRIC_GRID+cv::CALIB_CB_CLUSTERING);
    
    //std::cout << "No. of corners detected:- " << centers.size() << std::endl;
    //std::cout << "Co-ordinate of top left corner:- " << centers[0].x << " " << centers[0].y << std::endl;
    
    if (drawCenters) {
        cv::drawChessboardCorners(dst, cv::Size(4, 11), centers, found);
    }
    
    return(found);
}


/*
 Given a vector of points having image pixel coordinates of detected circle centers,
 this function populates the vector of points in world coordinates for the checkerboard target.
 It also populates the vectors for storing multiple point sets and center sets to be used in calibration.
 */
int selectCalibration(std::vector<cv::Point2f> &corners, std::vector<std::vector<cv::Point2f>> &corners_list, std::vector<cv::Vec3f> &points, std::vector<std::vector<cv::Vec3f>> &points_list)
{
    points.push_back(cv::Vec3f(10, 7, 0));
    points.push_back(cv::Vec3f(10, 5, 0));
    points.push_back(cv::Vec3f(10, 3, 0));
    points.push_back(cv::Vec3f(10, 1, 0));
    points.push_back(cv::Vec3f(9, 6, 0));
    points.push_back(cv::Vec3f(9, 4, 0));
    points.push_back(cv::Vec3f(9, 2, 0));
    points.push_back(cv::Vec3f(9, 0, 0));
    points.push_back(cv::Vec3f(8, 7, 0));
    points.push_back(cv::Vec3f(8, 5, 0));
    points.push_back(cv::Vec3f(8, 3, 0));
    points.push_back(cv::Vec3f(8, 1, 0));
    points.push_back(cv::Vec3f(7, 6, 0));
    points.push_back(cv::Vec3f(7, 4, 0));
    points.push_back(cv::Vec3f(7, 2, 0));
    points.push_back(cv::Vec3f(7, 0, 0));
    points.push_back(cv::Vec3f(6, 7, 0));
    points.push_back(cv::Vec3f(6, 5, 0));
    points.push_back(cv::Vec3f(6, 3, 0));
    points.push_back(cv::Vec3f(6, 1, 0));
    points.push_back(cv::Vec3f(5, 6, 0));
    points.push_back(cv::Vec3f(5, 4, 0));
    points.push_back(cv::Vec3f(5, 2, 0));
    points.push_back(cv::Vec3f(5, 0, 0));
    points.push_back(cv::Vec3f(4, 7, 0));
    points.push_back(cv::Vec3f(4, 5, 0));
    points.push_back(cv::Vec3f(4, 3, 0));
    points.push_back(cv::Vec3f(4, 1, 0));
    points.push_back(cv::Vec3f(3, 6, 0));
    points.push_back(cv::Vec3f(3, 4, 0));
    points.push_back(cv::Vec3f(3, 2, 0));
    points.push_back(cv::Vec3f(3, 0, 0));
    points.push_back(cv::Vec3f(2, 7, 0));
    points.push_back(cv::Vec3f(2, 5, 0));
    points.push_back(cv::Vec3f(2, 3, 0));
    points.push_back(cv::Vec3f(2, 1, 0));
    points.push_back(cv::Vec3f(1, 6, 0));
    points.push_back(cv::Vec3f(1, 4, 0));
    points.push_back(cv::Vec3f(1, 2, 0));
    points.push_back(cv::Vec3f(1, 0, 0));
    points.push_back(cv::Vec3f(0, 7, 0));
    points.push_back(cv::Vec3f(0, 5, 0));
    points.push_back(cv::Vec3f(0, 3, 0));
    points.push_back(cv::Vec3f(0, 1, 0));
    
    corners_list.push_back(corners);
    points_list.push_back(points);
    
    return(0);
}


/*
 Given vectors having a list of point sets and center sets, an initial camera matrix,
 this function generates the calibration and calculates the calibrated camera matrix and distortion coeffecients.
 This function also returns the reprojection error after performing calibration.
 */
float calibrateCamera(std::vector<std::vector<cv::Vec3f>> &points_list, std::vector<std::vector<cv::Point2f>> &centers_list, cv::Mat &camera_matrix, cv::Mat &dist_coeff)
{
    std::vector<cv::Mat> rot, trans;
    
    float error = cv::calibrateCamera(points_list,
                                centers_list,
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
 Given camera matrix and distance coefficients,
 this function saves the current calibration into a CSV file to be retrieved later for calculating camera pose.
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
 Given vector containing current point set and center set, calibrated camera matrix and distortion coeffcients,
 this function estimnates the position of the camera relative to the target and populates arrays with rotation and translation data.
 */
int calcCameraPosition(std::vector<cv::Vec3f> &points, std::vector<cv::Point2f> &centers, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans)
{
    cv::solvePnP(points, centers, camera_matrix, dist_coeff, rot, trans);
    
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
    points.push_back(cv::Vec3f({0, 2, 0}));
    points.push_back(cv::Vec3f({0, 0, 2}));
    
    std::vector<cv::Point2f> centers;
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);
    
    cv::arrowedLine(src, centers[0], centers[1], cv::Scalar(0, 0, 255), 5);
    cv::arrowedLine(src, centers[0], centers[2], cv::Scalar(0, 255, 0), 5);
    cv::arrowedLine(src, centers[0], centers[3], cv::Scalar(255, 0, 0), 5);
    
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
    points.push_back(cv::Vec3f({2, 2, 3})); // center
    points.push_back(cv::Vec3f({3, 1, 0})); // tr
    points.push_back(cv::Vec3f({3, 3, 0})); // br
    points.push_back(cv::Vec3f({1, 3, 0})); // bl
    points.push_back(cv::Vec3f({1, 1, 0})); // tl
    
    std::vector<cv::Point2f> centers;
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);

    cv::line(src, centers[0], centers[1], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[0], centers[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[0], centers[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[0], centers[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[1], centers[2], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[2], centers[3], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[3], centers[4], cv::Scalar(0, 255, 255), 5);
    cv::line(src, centers[4], centers[1], cv::Scalar(0, 255, 255), 5);
        
    points.clear();
    centers.clear();
    
    // CUBE
    points.push_back(cv::Vec3f({8, 5, 0})); // br
    points.push_back(cv::Vec3f({8, 5, 2}));
    points.push_back(cv::Vec3f({6, 5, 0})); // bl
    points.push_back(cv::Vec3f({6, 5, 2}));
    points.push_back(cv::Vec3f({8, 3, 0})); // tr
    points.push_back(cv::Vec3f({8, 3, 2}));
    points.push_back(cv::Vec3f({6, 3, 0})); // tl
    points.push_back(cv::Vec3f({6, 3, 2}));
    
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);

    cv::line(src, centers[0], centers[2], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[4], centers[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[0], centers[4], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[2], centers[6], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[1], centers[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[5], centers[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[1], centers[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[3], centers[7], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[0], centers[1], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[2], centers[3], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[4], centers[5], cv::Scalar(255, 255, 0), 5);
    cv::line(src, centers[6], centers[7], cv::Scalar(255, 255, 0), 5);

    return(0);
}


/*
 Given a cv::Mat of the image frame, a cv::Mat of the output frame, calibrated camera matrix, distortion coefficients, rotation & translation data and filename for artwork image,
 this function reads the artwork image and draws artwork image on the target using perspective transformation.
 */
int drawOnTarget(cv::Mat &src, cv::Mat &dst, cv::Mat &camera_matrix, cv::Mat &dist_coeff, cv::Mat &rot, cv::Mat &trans, std::string img_filename)
{
    cv::Point2f inputQuad[4];
    cv::Point2f outputQuad[4];
    
    cv::Mat lambda(2, 4, CV_32FC1);
    cv::Mat canvas;
    
    canvas = cv::imread(img_filename, 1);
    
    lambda = cv::Mat::zeros(canvas.rows, canvas.cols, canvas.type());
    
    inputQuad[0] = cv::Point2f(0, 0);
    inputQuad[1] = cv::Point2f(canvas.cols, 0);
    inputQuad[2] = cv::Point2f(canvas.cols-1, canvas.rows-1);
    inputQuad[3] = cv::Point2f(0, canvas.rows-1);
    
    
    std::vector<cv::Vec3f> points;
    points.push_back(cv::Vec3f({-3, 9, 0}));
    points.push_back(cv::Vec3f({13, 9, 0}));
    points.push_back(cv::Vec3f({13, -2, 0}));
    points.push_back(cv::Vec3f({-3, -2, 0}));
    
    std::vector<cv::Point2f> centers;
        
    cv::projectPoints(points, rot, trans, camera_matrix, dist_coeff, centers);
    
    outputQuad[0] = centers[0];
    outputQuad[1] = centers[1];
    outputQuad[2] = centers[2];
    outputQuad[3] = centers[3];
    
    std::vector<cv::Point> vertices{outputQuad[0], outputQuad[1], outputQuad[2], outputQuad[3]};
    std::vector<std::vector<cv::Point>> pts{vertices};
    cv::fillPoly(src, pts, cv::Scalar(0, 0, 0));
    
    lambda = getPerspectiveTransform(inputQuad, outputQuad);
    warpPerspective(canvas, dst, lambda, dst.size());
    
    src.copyTo(dst, src);
    
    return(0);
}
