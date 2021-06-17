#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <unistd.h>
#include <iostream>
#include <vector>

void calibrate_color_ir(cv::Mat &H_rgb, cv::Mat &H_ir, rs2::pipeline &pipe, const char* window_name_color, const char* window_name_ir);