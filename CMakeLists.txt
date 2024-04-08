cmake_minimum_required(VERSION 3.10)
project(StereoCalibration)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add executable
add_executable(StereoCalibration src/main.cpp)

# Link libraries
target_link_libraries(StereoCalibration ${OpenCV_LIBS})