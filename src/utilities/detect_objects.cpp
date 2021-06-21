#include "detect_objects.hpp"

void detect_objects(Mat &img, Mat &img_ground, Rect &roi, int th, vector<Point2f> &centroids_out) {
    cv::Mat img_sub;
    cv::subtract(img_ground, img, img_sub);
    cv::cvtColor(img_sub, img_sub, cv::COLOR_RGB2GRAY);

    img_sub = img_sub(roi);

    cv::medianBlur(img_sub, img_sub, 5);

    cv::Mat bw_img;
    cv::threshold(img_sub, bw_img, 10, 255, cv::THRESH_BINARY);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    // find contours
    findContours( bw_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    // get the moments
    vector<Moments> mu(contours.size());
    for( int i = 0; i<contours.size(); i++ ) { 
        mu[i] = moments( contours[i], false ); 
    }

    // get the centroid of figures.
    vector<Point2f> mc(contours.size());
    for( int i = 0; i<contours.size(); i++) { 
        mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
    }

    // draw contours
    Mat drawing(bw_img.size(), CV_8UC3, Scalar(255,255,255));
    vector<Point2f> centroids(contours.size());

    int n_objects = 0;

    for( int i = 0; i<contours.size(); i++ ) {
        if(contourArea(contours[i]) > 100) {
            Scalar color = Scalar(167,151,0); // B G R values
            drawContours(img, contours, i, color, 2, 8, hierarchy, 0, Point());
            circle( img, mc[i], 4, color, -1, 8, 0 );
            std::cout << "Centroid " << i << ": (" << mc[i].x << ", " << mc[i].y << ")" << std::endl;
            centroids[n_objects] = mc[i];
            n_objects++;
        }
    }
    centroids_out.resize(n_objects);
    for(int i=0; i<n_objects; i++) {
        centroids_out[i] = centroids[i];
        std::cout << "Centroid " << i << ": (" << centroids[i].x << ", " << centroids[i].y << ")" << std::endl;
    }

}