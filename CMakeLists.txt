cmake_minimum_required(VERSION 3.10)
project(StereoCalibration)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find OpenCV package
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

# Add the AprilTags library
add_subdirectory(apriltags)  # Adjust the path if necessary

# Include directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    apriltags
    ${EIGEN3_INCLUDE_DIR}
    include
)

# Add executable for the main project
add_executable(StereoCalibration src/main.cpp)

# Link libraries with both OpenCV and AprilTags
target_link_libraries(StereoCalibration ${OpenCV_LIBS} apriltags Eigen3::Eigen)