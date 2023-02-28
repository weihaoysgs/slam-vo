cmake_minimum_required(VERSION 3.10)

project(slam-vo)

set(CMAKE_BUILD_TYPE Release)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_CXX_STANDARD 14)

############### dependencies ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
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

set(THIRD_PARTY_LIBS
${OpenCV_LIBS}
${Sophus_LIBRARIES}
${Pangolin_LIBRARIES} GL GLU GLEW glut
g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
${GTEST_BOTH_LIBRARIES}
${GLOG_LIBRARIES}
${GFLAGS_LIBRARIES}
pthread
${CSPARSE_LIBRARY}
fmt::fmt
)

set(SRC_FILES_LIBS
src/Config.cpp
src/Dataset.cpp
src/Camera.cpp
src/Frame.cpp
src/VisualOdometry.cpp
src/Frontend.cpp
)


SET(TEST_SOURCES camera_test config_test dataset_test vo_test)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} test/${test_src}.cpp ${SRC_FILES_LIBS})
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS})
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)


add_executable(run_kitti_stereo
./main/run_kitti_stereo.cpp
${SRC_FILES_LIBS}
)
target_link_libraries(run_kitti_stereo
${THIRD_PARTY_LIBS}
)
