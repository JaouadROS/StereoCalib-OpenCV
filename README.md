# StereoCalib-OpenCV using Chessboard

StereoCalib-OpenCV is a comprehensive toolkit for stereo camera calibration using OpenCV, aimed at facilitating accurate 3D reconstruction and depth perception. This project provides easy-to-use tools and examples for performing stereo camera calibration, optimizing camera parameters, and saving these parameters for future use in stereo vision applications.

## Features

- Stereo camera calibration using checkerboard patterns.
- Calculation of intrinsic and extrinsic parameters.
- Optimization of camera matrices for better undistortion and rectification.
- Saving and loading of calibration parameters in XML format.

## Getting Started

### Prerequisites

- OpenCV (Version 4.x recommended)

### Installation

1. Clone this repository to your local machine:
    ```
    git clone https://github.com/JaouadROS/StereoCalib-OpenCV.git
    ```
2. Navigate to the cloned repository directory:
    ```
    cd StereoCalib-OpenCV
    ```
3. Create a build directory and navigate into it:
    ```
    mkdir build && cd build
    ```
4. Run CMake to configure the project:
    ```
    cmake ..
    ```
5. Build the project:
    ```
    make -j
    ```

### Usage

1. Place your stereo camera images in the `data/stereoL` and `data/stereoR` directories for left and right images, respectively.
2. Run the compiled executable to start the calibration process:
    ```
    ./StereoCalibration
    ```
3. Upon completion, the output file containing the calibration parameters will be saved in the `data` folder.

