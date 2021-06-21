# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
cmake_minimum_required(VERSION 3.1.0)

project(3d_object_localization)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_INCLUDE_PATH "/usr/local/include")
find_package (Eigen3 3.3 REQUIRED NO_MODULE)



include_directories(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Add the application sources to the target
add_executable(rs_3d_localization src/main.cpp src/utilities/calibrate_rs.cpp)

add_executable(rs_3d_get_imgs src/store_img.cpp)
add_executable(rs_offline src/main_offline_ir.cpp src/utilities/detect_objects src/utilities/homography_rs)


# Link librealsense2 to the target
target_link_libraries(rs_3d_localization ${realsense2_LIBRARY} ${OpenCV_LIBS})
target_link_libraries(rs_3d_get_imgs ${realsense2_LIBRARY} ${OpenCV_LIBS})
target_link_libraries(rs_offline ${realsense2_LIBRARY} ${OpenCV_LIBS})
