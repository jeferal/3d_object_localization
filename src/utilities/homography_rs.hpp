#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/calib3d/calib3d.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

void homography_rs(Mat &img_calibration, Mat &H);
