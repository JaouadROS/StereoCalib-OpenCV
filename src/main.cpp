#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <StereoCalib-OpenCV/demo.hpp>

// Parameters for the board layout
const int tagsPerRow = 6;
const int tagsPerCol = 6;
const float tagSize = 0.05f; // eg. tag is 5cm x 5cm
const float tagSpacing = 0.005f; // eg. 0.5cm gap between tags

int main(int argc, char* argv[])
{
    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;

    objp.reserve(tagsPerRow * tagsPerCol * 4); // 4 corners per tag
    for (int i = 0; i < tagsPerCol; ++i) { // For each row
        for (int j = 0; j < tagsPerRow; ++j) { // For each column
            // Calculate the base position of this tag
            float xBase = j * (tagSize + tagSpacing);
            float yBase = i * (tagSize + tagSpacing);

            // Top-left corner
            objp.emplace_back(cv::Point3f(xBase, yBase, 0.0f));
            // Top-right corner
            objp.emplace_back(cv::Point3f(xBase + tagSize, yBase, 0.0f));
            // Bottom-right corner
            objp.emplace_back(cv::Point3f(xBase + tagSize, yBase + tagSize, 0.0f));
            // Bottom-left corner
            objp.emplace_back(cv::Point3f(xBase, yBase + tagSize, 0.0f));
        }
    }
    std::cout<<"objp size: "<<objp.size()<<std::endl;

    // AprilTags demo object setup
    Demo demo;
    demo.parseOptions(argc, argv);
    demo.setup();

    // Creating vector to store vectors of 3D points for each aprilboard image
    std::vector<std::vector<cv::Point3f> > objpoints;

    // Creating vector to store vectors of 2D points for each aprilboard image
    std::vector<std::vector<cv::Point2f> > imgpointsL, imgpointsR;

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;

    // Path of the folder containing aprilboard images
    std::string pathL = argv[1]; 
    std::string pathR = argv[2];

    std::cout<<"pathL: "<<pathL<<std::endl;
    std::cout<<"pathR: "<<pathR<<std::endl;

    cv::glob(pathL, imagesL);
    cv::glob(pathR, imagesR);

    cv::Mat frameL, frameR, grayL, grayR;
    // Looping over all the images in the directory
    for(int i{0}; i<imagesL.size(); i++)
    {
        // vector to store the pixel coordinates of detected checker board corners 
        std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
        bool successL, successR;

        frameL = cv::imread(imagesL[i]);
        cv::cvtColor(frameL,grayL,cv::COLOR_BGR2GRAY);

        frameR = cv::imread(imagesR[i]);
        cv::cvtColor(frameR,grayR,cv::COLOR_BGR2GRAY);

        // Finding tags corners
        // If desired number of corners are found in the image then success = true  
        vector<AprilTags::TagDetection> detectionsL = demo.extractTags(grayL);
        successL = detectionsL.size() == tagsPerRow * tagsPerCol;
        for (int i=0; i<detectionsL.size(); i++) {
            detectionsL[i].draw(frameL);
        }
        for(const auto& detection : detectionsL) {
            for(int cornerIndex = 0; cornerIndex < 4; cornerIndex++) {
                corner_ptsL.push_back(cv::Point2f(detection.p[cornerIndex].first, detection.p[cornerIndex].second));
            }
        }

        vector<AprilTags::TagDetection> detectionsR = demo.extractTags(grayR);
        successR = detectionsR.size() == tagsPerRow * tagsPerCol;
        for(int i=0; i<detectionsR.size(); i++) {
            detectionsR[i].draw(frameR);
        }
        for(const auto& detection : detectionsR) {
            for(int cornerIndex = 0; cornerIndex < 4; cornerIndex++) {
                corner_ptsR.push_back(cv::Point2f(detection.p[cornerIndex].first, detection.p[cornerIndex].second));
            }
        }

        /*
            * If desired number of corner are detected,
            * we refine the pixel coordinates and display 
            * them on the images of checker board
        */
        if((successL) && (successR))
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL,corner_ptsL,cv::Size(11,11), cv::Size(-1,-1),criteria);
            cv::cornerSubPix(grayR,corner_ptsR,cv::Size(11,11), cv::Size(-1,-1),criteria);

            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }

        cv::imshow("ImageL",frameL);
        cv::imshow("ImageR",frameR);
        cv::waitKey(10);
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