#include "homography_rs.hpp"

void homography_rs(Mat &img_calibration, Mat &H) {
    cv::Size patternSize(9,6);

    //Get chess board corners (RGB)
    std::vector<cv::Point2f> points_cam;
    bool is_found = cv::findChessboardCorners(img_calibration, patternSize, points_cam);

    std::cout << img_calibration.size() << std::endl;
    if(is_found) {
        std::vector<cv::Point3f> real_world_points;
        std::vector<cv::Point2f> real_world_points_2d;

        float squareSize = 1.0; //unit
        for( int i = 1; i <= 6; i++ ) {
            for( int j = 1; j <= 9; j++ ) {
                real_world_points_2d.push_back(cv::Point2f(float(j*squareSize),float(i*squareSize)));
                real_world_points.push_back(cv::Point3f(float(j*squareSize),float(i*squareSize), 0));
            }
        }

        //Homography between color camera and chessboard plane
        H = cv::findHomography(real_world_points, points_cam);
        std::cout << "Homography found" << std::endl;
        std::cout << H << std::endl;
        cv::drawChessboardCorners(img_calibration, patternSize, points_cam, is_found);

        //Camera calibration
        cv::Mat cameraMatrix;
        vector<Mat> rvecs, tvecs;
        vector<float> reprojErrs;
        cv::Mat distCoeffs;
        cv::Size img_size = img_calibration.size();

        std::vector<std::vector<cv::Point2f> > imagePoints;
        imagePoints.push_back(points_cam);

        cv::Size win_size(5,5);
        cv::Size zero_zone(-1,-1);
        TermCriteria criteria = TermCriteria( TermCriteria::EPS + TermCriteria::COUNT, 40, 0.001 );
        cornerSubPix(img_calibration, points_cam, win_size, zero_zone, criteria);

        for( int i = 0; i < 54; i++ ) {
            real_world_points[i] = (cv::Point3f(real_world_points_2d[i].x, real_world_points_2d[i].y, 0));
        }

        std::vector<std::vector<cv::Point3f> > objectPoints;
        objectPoints.push_back(real_world_points);

        std::cout << "Real world" << real_world_points << real_world_points.size() << std::endl;
        std::cout << "Img points" << points_cam << points_cam.size() << std::endl;
   
        double rms = calibrateCamera(objectPoints, imagePoints, img_size, cameraMatrix,
                            distCoeffs, rvecs, tvecs);

        cv::Mat rot_vec = Mat::zeros(1,3,CV_64F);
        std::cout << "A matrix: " << cameraMatrix << std::endl;
        std::cout << "Rvecs[0]: " << rvecs[0];
        Rodrigues(rvecs[0], rot_vec);
        cv::Mat P_matrix(cv::Size(3, 4), CV_64F);
        cv::Mat r_t_;
        hconcat(rot_vec, tvecs[0], r_t_);
        std::cout << "Concatenation rot_vec, tvecs[0]" << r_t_ << std::endl;
        P_matrix = cameraMatrix * r_t_;
        std::cout << "P matrix: " << std::endl;
        std::cout << P_matrix << std::endl;
    } else {
        std::cerr << "No chessboard found" << std::endl;
    }
}
