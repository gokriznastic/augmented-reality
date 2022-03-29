/*
Gopal Krishna

CS 5330 Computer Vision
Spring 2021
Project 4

main() CPP function for extensions to the AR system.
*/

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "extend_helper.h"

/*
 main function
 */
int main(int argc, char *argv[])
{
    cv::VideoCapture *capdev;

    // open the video device
    capdev = new cv::VideoCapture(0);
    if(!capdev->isOpened())
    {
        printf("Unable to open video device\n");
        return(-1);
    }

    // get some properties of the image
    capdev -> set(cv::CAP_PROP_FRAME_WIDTH,  960);
    capdev -> set(cv::CAP_PROP_FRAME_HEIGHT, 540);
    cv::Size refS((int) capdev->get(cv::CAP_PROP_FRAME_WIDTH),
                  (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    cv::namedWindow("Video", 1); // identifies a window
    
    // initialize global variables for different tasks
    cv::Mat frame;
    int frameNo = 1;
    int calFrameNo = 1;
    cv::Mat output;
    
    bool drawCenters = true;
    std::vector<std::vector<cv::Vec3f>> points_list;
    std::vector<std::vector<cv::Point2f>> centers_list;
    
    double cammat[] = {1, 0, (double) frame.cols/2, 0, 1, (double) frame.rows/2, 0, 0, 1};
    cv::Mat camera_matrix(cv::Size(3,3), CV_64FC1, &cammat);
    cv::Mat dist_coeff;
    cv::Mat rot, trans;
    bool showAxes = false;
    bool showObject = false;
    
    bool canvas = false;
        
    // start live feed from the video device
    while(true) {
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if(frame.empty()) {
          printf("frame is empty\n");
          break;
        }
        
        std::vector<cv::Point2f> centers;
        std::vector<cv::Vec3f> points;
        
        // extract corners from circle-grid
        bool found = extractCircleCenters(frame, output, centers, drawCenters);
        
        // display axes
        if (showAxes && found) {
            selectCalibration(centers, centers_list, points, points_list);

            // calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coeff, rot, trans);
            std::cout << std::endl << "rotation matrix: " << rot << std::endl;
            std::cout << std::endl << "translation matrix: " << trans << std::endl;

            // project 3D axes
            draw3dAxes(output, camera_matrix, dist_coeff, rot, trans);
        }

        // display virtual object
        if (showObject && found) {
            selectCalibration(centers, centers_list, points, points_list);

            // calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coeff, rot, trans);
            std::cout << std::endl << "rotation matrix: " << rot << std::endl;
            std::cout << std::endl << "translation matrix: " << trans << std::endl;

            // create a virtual object
            draw3dObject(output, camera_matrix, dist_coeff, rot, trans);
        }
        
        // transform target into image canvas
        if (canvas && found) {
            selectCalibration(centers, centers_list, points, points_list);

            // calculate current position of the camera
            calcCameraPosition(points, centers, camera_matrix, dist_coeff, rot, trans);
            std::cout << std::endl << "rotation matrix: " << rot << std::endl;
            std::cout << std::endl << "translation matrix: " << trans << std::endl;
            
            std::string imageFilename = "gogh.jpeg";
            // draw image contents on the target
            drawOnTarget(frame, output, camera_matrix, dist_coeff, rot, trans, imageFilename);
        }
        
        // display the current frame
        cv::imshow("Video", output);

        // see if there is a waiting keystroke
        char key = cv::waitKey(10);
        
        // press 'q' to quit
        if(key == 'q') {
            break;
        }
        // press 's' to save current calibration frame and perform calibration if frames >= 5
        else if(key == 's' && found && !showAxes && !showObject && drawCenters) {
            // select calibration images
            selectCalibration(centers, centers_list, points, points_list);

            printf("Saving calibration image...\n");
            std::string fname = "calibration-frame-";
            fname += std::to_string(calFrameNo) + ".jpg";
            imwrite(fname, output);

            // print the corner points in world coordinates with corresponding image coordinates
            std::cout << "---------------------------------------------------------------------------" << std::endl;
            std::cout << "Calibration image " << calFrameNo << " saved" << std::endl;
            std::cout << "Point set " << calFrameNo << " \t Corners set " << calFrameNo << std::endl;
            for(int i=0; i<points.size(); i++) {
                cv::Vec3s point = points[i];
                cv::Point2f corner = centers[i];
                printf("[%d %d %d] \t\t", point[0], point[1], point[2]);
                printf("[%f, %f] \n", corner.x, corner.y);
            }
            std::cout << "---------------------------------------------------------------------------" << std::endl;

            // require at least 5 frames for calibration
            if (calFrameNo >= 5) {
                // calibrate the camera
                std::cout << "Performing calibration with " << calFrameNo << " frames..." << std::endl;
                std::cout << "initial camera matrix:" << std::endl;
                std::cout << camera_matrix << std::endl;

                float reprojErr = calibrateCamera(points_list, centers_list, camera_matrix, dist_coeff);

                // print the calibration stats for the user
                std::cout << "calibrated camera matrix:" << std::endl;
                std::cout << camera_matrix << std::endl;
                std::cout << "re-projection error: " << reprojErr << std::endl;
                std::cout << "distortion coefficients: " << dist_coeff << std::endl;
            }
            
            calFrameNo++;
        }
        // press 'c' to save current calibration in a csv file to be read later
        else if (key == 'c' && found && !showAxes && !showObject && drawCenters) {
            // save current calibration in a csv file
            std::cout << std::endl << "Saving performed calibration..." << std::endl;
            saveCalibration(camera_matrix, dist_coeff);
        }
        // press 'x' to display 3d axes at the origin of world coordinates
        else if (key == 'x' && found) {
            showAxes = !showAxes;
            if (showObject) {
                showObject = !showObject;
            }
            drawCenters = !(showAxes || showObject);
            if (canvas) {
                canvas = !canvas;
            }
            
            // read calibration to display axes
            std::string fileName = "intrinsic_data_circlegrid.csv";
            readCalibration(fileName, camera_matrix, dist_coeff);
            std::cout << std::endl << "retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }
        // press 'o' to display 3d objects
        else if (key == 'o' && found) {
            if(showAxes) {
                showAxes = !showAxes;
            }
            showObject = !showObject;
            drawCenters = !(showObject || showAxes);
            if (canvas) {
                canvas = !canvas;
            }
            
            // read calibration to display virtual object
            std::string fileName = "intrinsic_data_circlegrid.csv";
            readCalibration(fileName, camera_matrix, dist_coeff);
            std::cout << std::endl << "retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }
        // press 't' to place image canvas on target
        else if (key == 't' && found) {
            canvas = !canvas;
            if(showAxes) {
                showAxes = !showAxes;
            }
            if (showObject) {
                showObject = !showObject;
            }
            drawCenters = !(showObject || showAxes || canvas);
            
            // read calibration to transform target
            std::string fileName = "intrinsic_data_circlegrid.csv";
            readCalibration(fileName, camera_matrix, dist_coeff);
            std::cout << std::endl << "retrieved calibrated camera matrix:" << std::endl;
            std::cout << camera_matrix << std::endl;
            std::cout << "distortion coefficients: " << dist_coeff << std::endl;
        }
        // press 'p' to take a snapshot of the current frame
        else if(key == 'p') {
            printf("saving image\n");
            std::string fname = "frame-";
            fname += std::to_string(frameNo) + ".jpg";
            imwrite(fname, output);
            frameNo++;
        }
    }

    delete capdev;
    
    return(0);
}
