#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API

#include <unistd.h>
#include <iostream>
#include <vector>

#include "utilities/calibrate_rs.hpp"


int main(int argc, char * argv[]) {

    std::cout << "Starting program" << std::endl;

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

    cv::Mat H;
    calibrate_color_ir(H, pipe);

    std::cout << H << std::endl;

    pipe.stop();

    return 0;

}