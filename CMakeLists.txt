cmake_minimum_required(VERSION 3.10)

project(slam-vo)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 14)

############### dependencies ######################
include_directories("/usr/include/eigen3")
find_package(OpenCV 3.2.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(Glog REQUIRED)
find_package(GTest REQUIRED)
find_package(GFlags REQUIRED)

find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GFLAGS_INCLUDE_DIRS})
include_directories(${GLOG_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(run_kitti_stereo
./main/run_kitti_stereo.cpp
)
