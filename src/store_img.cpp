#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>

int main(int argc, char * argv[]) {
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 360, RS2_FORMAT_BGR8, 30);
    
    // Start streaming with default recommended configuration
    rs2::pipeline_profile prof = pipe.start(cfg);

    rs2::device dev = prof.get_device();

    auto depth_sensor = dev.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    }

    std::cout << "Pipeline started" << std::endl;

    const auto window_name_color = "Display Image Color Cam";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);
    const auto window_name_ir = "Display Image IR Cam";
    cv::namedWindow(window_name_ir, cv::WINDOW_AUTOSIZE);

    int iter=0;
    while(true) {
        std::cout << "New frame" << std::endl;
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        auto rgb_frame = data.get_color_frame();
        auto ir_frame = data.get_infrared_frame(1);

        cv::Mat image(cv::Size(640, 360), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat ir_img(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

        std::cout << "Store imgs? (space/0) esc for exit";

        cv::Size patternSize(9,6);
        std::vector<cv::Point2f> points_color_cam;
        cv::Mat img_ir_points = ir_img.clone();
        bool is_found = cv::findChessboardCorners(ir_img, patternSize, points_color_cam);
        std::cout << is_found << std::endl;
        if(is_found) {
            cv::drawChessboardCorners(img_ir_points, patternSize, points_color_cam, is_found);
        }

        cv::imshow(window_name_ir, img_ir_points);
        cv::imshow(window_name_color, image);

        int k = cv::waitKey();
        if(k==27)
            break;

        if(k==32) {
            std::string img_color_name = "/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_3/img_color_figures" + std::to_string(iter) + ".jpg";
            std::string img_ir_name = "/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_3/img_ir_figures" + std::to_string(iter) + ".jpg";
            cv::imwrite(img_color_name, image);
            cv::imwrite(img_ir_name, ir_img);
            iter++;
        }

    }

    return 0;
}