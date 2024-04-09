#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <stdio.h>
#include <iostream>
#include <vector>

// Defining the dimensions of checkerboard
int CHECKERBOARD[2]{6,9}; 

int main()
{
    // Define your ArUco dictionary here (DICT_APRILTAG_36h11 is an example)
    cv::Ptr<cv::aruco::Dictionary> arucoDict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::DetectorParameters> arucoParams = cv::aruco::DetectorParameters::create();
    arucoParams->markerBorderBits = 2;

    // Storage for 3D points and 2D corner points
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpointsL, imgpointsR;

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;
    std::string pathL = "../data/stereoL/*.png";
    std::string pathR = "../data/stereoR/*.png";

    cv::glob(pathL, imagesL);
    cv::glob(pathR, imagesR);

    cv::Mat grayL, grayR;
    for (size_t i = 0; i < imagesL.size(); i++) {
        cv::Mat frameL = cv::imread(imagesL[i]);
        cv::Mat frameR = cv::imread(imagesR[i]);

        cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
        cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);

        std::vector<int> idsL, idsR;
        std::vector<std::vector<cv::Point2f>> cornersL, cornersR;

        // Detect ArUco markers in the left and right images
        cv::aruco::detectMarkers(grayL, arucoDict, cornersL, idsL, arucoParams);
        cv::aruco::detectMarkers(grayR, arucoDict, cornersR, idsR, arucoParams);

        // If markers are found in both images
        if (!idsL.empty() && !idsR.empty()) {
            // Example (pseudo-code, adapt based on your setup):
            // for each detected marker ID in idsL and idsR, find its corresponding 3D position,
            // and add to objpoints. Similarly, add detected 2D corners to imgpointsL and imgpointsR.

            // Draw detected markers on the left and right images
            cv::aruco::drawDetectedMarkers(frameL, cornersL, idsL);
            cv::aruco::drawDetectedMarkers(frameR, cornersR, idsR);
        }

        cv::imshow("ImageL", frameL);
        cv::imshow("ImageR", frameR);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();

    cv::Mat mtxL,distL,R_L,T_L;
    cv::Mat mtxR,distR,R_R,T_R;


    /*
    * Performing camera calibration by 
    * passing the value of known 3D points (objpoints)
    * and corresponding pixel coordinates of the 
    * detected corners (imgpoints)
    */

    cv::Mat new_mtxL, new_mtxR;

    // Calibrating left camera
    double rmsL = cv::calibrateCamera(objpoints,
                        imgpointsL,
                        grayL.size(),
                        mtxL,
                        distL,
                        R_L,
                        T_L);

    new_mtxL = cv::getOptimalNewCameraMatrix(mtxL,
                                distL,
                                grayL.size(),
                                1,
                                grayL.size(),
                                0);

    // Calibrating right camera
    double rmsR = cv::calibrateCamera(objpoints,
                        imgpointsR,
                        grayR.size(),
                        mtxR,
                        distR,
                        R_R,
                        T_R);

    new_mtxR = cv::getOptimalNewCameraMatrix(mtxR,
                                distR,
                                grayR.size(),
                                1,
                                grayR.size(),
                                0);

    // Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat 
    // are calculated. Hence intrinsic parameters are the same.
    cv::Mat Rot, Trns, Emat, Fmat;

    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;


    // This step is performed to transformation between the two cameras and calculate Essential and 
    // Fundamenatl matrix
    double rms = cv::stereoCalibrate(objpoints,
                        imgpointsL,
                        imgpointsR,
                        new_mtxL,
                        distL,
                        new_mtxR,
                        distR,
                        grayR.size(),
                        Rot,
                        Trns,
                        Emat,
                        Fmat,
                        flag,
                        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;

    // Once we know the transformation between the two cameras we can perform 
    // stereo rectification
    cv::stereoRectify(new_mtxL,
                    distL,
                    new_mtxR,
                    distR,
                    grayR.size(),
                    Rot,
                    Trns,
                    rect_l,
                    rect_r,
                    proj_mat_l,
                    proj_mat_r,
                    Q,
                    1);

    // Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
    // Compute the rectification map (mapping between the original image pixels and 
    // their transformed values after applying rectification and undistortion) for left and right camera frames
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::initUndistortRectifyMap(new_mtxL,
                                distL,
                                rect_l,
                                proj_mat_l,
                                grayR.size(),
                                CV_16SC2,
                                Left_Stereo_Map1,
                                Left_Stereo_Map2);

    cv::initUndistortRectifyMap(new_mtxR,
                                distR,
                                rect_r,
                                proj_mat_r,
                                grayR.size(),
                                CV_16SC2,
                                Right_Stereo_Map1,
                                Right_Stereo_Map2);
  // Open file to save the parameters
    cv::FileStorage cv_file = cv::FileStorage("../data/params_cpp.xml", cv::FileStorage::WRITE);

    // Check if the file was successfully opened
    if (!cv_file.isOpened())
    {
        std::cerr << "Failed to open params_cpp.xml for writing." << std::endl;
        return -1;
    }

    // Write intrinsic parameters and projection matrices to the file
    cv_file << "rms" << rms;
    cv_file << "rmsL" << rmsL;
    cv_file << "MtxL" << mtxL;
    cv_file << "DistL" << distL;
    cv_file << "NewMtxL" << new_mtxL;
    cv_file << "rmsR" << rmsR;
    cv_file << "MtxR" << mtxR;
    cv_file << "DistR" << distR;
    cv_file << "NewMtxR" << new_mtxR;
    cv_file << "R_L" << R_L;
    cv_file << "T_L" << T_L;
    cv_file << "R_R" << R_R;
    cv_file << "T_R" << T_R;
    cv_file << "Rot" << Rot;
    cv_file << "Trns" << Trns;
    cv_file << "Emat" << Emat;
    cv_file << "Fmat" << Fmat;

    // Release the file
    cv_file.release();
 
  return 0;
}