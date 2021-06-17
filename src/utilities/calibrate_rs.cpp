#include "calibrate_rs.hpp"


void calibrate_color_ir(cv::Mat &H_rgb, cv::Mat &H_ir, rs2::pipeline &pipe) {

    //Exercise 1, find homography between color camera and chessboard plane
    bool calibrated = false;

    const auto window_name_color = "Display Image Color Cam";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);
    const auto window_name_ir = "Display Image IR Cam";
    cv::namedWindow(window_name_ir, cv::WINDOW_AUTOSIZE);


    while(!calibrated) {

        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        auto rgb_frame = data.get_color_frame();
        auto ir_frame = data.get_infrared_frame(1);

        cv::Mat image(cv::Size(640, 360), CV_8UC3, (void*)rgb_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat ir_img(cv::Size(1280, 720), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

        //Convert to gray scale
        cv::cvtColor(image, image, cv::COLOR_RGB2GRAY);

        cv::Size patternSize(9,6);

        //Get chess board corners (RGB)
        std::vector<cv::Point2f> points_color_cam;
        bool is_found_rgb = cv::findChessboardCorners(image, patternSize, points_color_cam);

        //Get chess board corners (IR)
        std::vector<cv::Point2f> points_ir_cam;
        bool is_found_ir = cv::findChessboardCorners(ir_img, patternSize, points_ir_cam);

        if(is_found_rgb && is_found_ir) {
            std::vector<cv::Point2f> real_world_points;

            float squareSize = 2.6; //cm
            for( int i = 0; i < 6; i++ ) {
                for( int j = 0; j < 9; j++ ) {
                    real_world_points.push_back(cv::Point2f(float(j*squareSize),float(i*squareSize)));
                }
            }

            //Homography between color camera and chessboard plane
            H_rgb = cv::findHomography(real_world_points, points_color_cam);

            H_ir = cv::findHomography(real_world_points, points_ir_cam);

            cv::drawChessboardCorners(image, patternSize, points_color_cam, is_found_rgb);
            cv::drawChessboardCorners(ir_img, patternSize, points_ir_cam, is_found_ir);

            std::vector<cv::Point2f> points_transformed;
            cv::perspectiveTransform(points_ir_cam, points_transformed, H_ir.inv());

            std::cout << points_transformed[4] << std::endl;

        }

        cv::imshow(window_name_color, image);
        cv::imshow(window_name_ir, ir_img);

        if(is_found_rgb) {
            int i=0;
            std::cout << "Calibrate (1/0)";
            std::cin >> i;
            if(i) break;
        }

        if (cv::waitKey(1) > 0) {
            break;
        }
    }

}