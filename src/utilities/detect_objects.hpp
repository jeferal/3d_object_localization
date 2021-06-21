#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include <unistd.h>
#include <iostream>
#include <vector>
#include <string>

using namespace cv;
using namespace std;

void detect_objects(Mat &img, Mat &img_ground, Rect &roi, int th, vector<Point2f> &centroids_out);