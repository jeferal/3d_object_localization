#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

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

    const auto window_name_color = "Display Image Color Cam";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);
    const auto window_name_ir = "Display Image IR Cam";
    cv::namedWindow(window_name_ir, cv::WINDOW_AUTOSIZE);

    //Get homography matrixes between chessboard plane and cameras
    cv::Mat H_rgb, H_ir;
    //calibrate_color_ir(H_rgb, H_ir, pipe, window_name_color, window_name_ir);

    std::cout << H_rgb << std::endl;
    std::cout << H_ir << std::endl;

    //Start object detection

    sleep(2);

    while(true) {
        std::cout << "New frame" << std::endl;
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        auto rgb_frame = data.get_color_frame();
        auto ir_frame = data.get_infrared_frame(1);

        cv::Mat image(cv::Size(640, 360), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat ir_img(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

        //Convert to gray scale
        cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

        cv::normalize(ir_img, ir_img, 0, 255, cv::NORM_MINMAX);

        double alpha = 2.0; /*< Simple contrast control */
        int beta = 50;       /*< Simple brightness control */
        for( int y = 0; y < ir_img.rows; y++ ) {
            for( int x = 0; x < ir_img.cols; x++ ) {
                for( int c = 0; c < ir_img.channels(); c++ ) {
                    ir_img.at<cv::Vec3b>(y,x)[c] =
                    cv::saturate_cast<uchar>( alpha*ir_img.at<cv::Vec3b>(y,x)[c] + beta );
                }
            }
        }

        int th=160;

        cv::imshow(window_name_color, ir_img);

        //Get contours
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours( ir_img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        ir_img = cv::Mat::zeros( ir_img.size(), CV_8UC3 );
        cv::RNG rng(12345);
        for( size_t i = 0; i< contours.size(); i++ ) {
            double area = cv::contourArea(contours[i]);
            std::cout << area << std::endl;
            if (area > 10000) {
                std::cout << "contour encoutered" << std::endl;
                //cv::minEnclosingCircle(contours[i], );
                cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
                drawContours( ir_img, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
            }
        }
        
        
        // Setup SimpleBlobDetector parameters.
        cv::SimpleBlobDetector::Params params;

        // Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 1000;
        params.maxArea = 10000;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;
        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); 

        // Detect blobs.
        std::vector<cv::KeyPoint> keypoints;
        detector->detect( ir_img, keypoints);
        
        int x_0 = 5, y_0 = 5;
        std::cout << keypoints.size() << std::endl;
        
        if(keypoints.size() > 1) {
            cv::rectangle(ir_img,cv::Point(keypoints[0].pt.x-x_0,keypoints[0].pt.y-y_0), 
                                    cv::Point(keypoints[1].pt.x+x_0,keypoints[1].pt.y +y_0),
                                    255,2);
        }
        
        cv::imshow(window_name_ir, ir_img);
        cv::waitKey(); 
    }

    pipe.stop();

    return 0;

}