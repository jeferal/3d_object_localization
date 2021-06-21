#include "utilities/detect_objects.hpp"
#include "utilities/homography_rs.hpp"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/SVD"

using namespace cv;
using namespace std;
using namespace Eigen;

int main() {
    //Get homographies
    cv::Mat calibration_color = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_3/img_color_figures0.jpg");
    cv::Mat calibration_ir = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_3/img_ir_figures0.jpg");

    Mat H_rgb;
    cvtColor(calibration_color, calibration_color, cv::COLOR_RGB2GRAY);
    homography_rs(calibration_color, H_rgb);
    const auto window_name_hm_color = "Homography RGB";
    cv::namedWindow(window_name_hm_color, cv::WINDOW_AUTOSIZE);
    imshow(window_name_hm_color, calibration_color);

    Mat H_ir;
    cvtColor(calibration_ir, calibration_ir, cv::COLOR_RGB2GRAY);
    homography_rs(calibration_ir, H_ir);
    const auto window_name_hm_ir = "Homography IR";
    cv::namedWindow(window_name_hm_ir, cv::WINDOW_AUTOSIZE);
    imshow(window_name_hm_ir, calibration_ir);
    
    
    cv::Mat img_ir = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_2/img_ir_figures0.jpg");
    cv::Mat img_ground_ir = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_2/img_ir_white0.jpg");

    cv::Mat img_color = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_2/img_color_figures0.jpg");
    cv::Mat img_ground_color = cv::imread("/Users/jeferal/OneDrive - UPV/universidad/MAI2/Visión artificial/lab/3D/project/object_3d_localization/imgs_2/img_color_white0.jpg");


    const auto window_name_ir = "Display Original IR img";
    cv::namedWindow(window_name_ir, cv::WINDOW_AUTOSIZE);

    const auto window_name_color = "Display Original COLOR img";
    cv::namedWindow(window_name_color, cv::WINDOW_AUTOSIZE);

    
    Rect roi_ir(0, 0, 800, 600);
    vector<Point2f> centroids_ir;
    detect_objects(img_ir, img_ground_ir, roi_ir, 100, centroids_ir);

    Rect roi_color(0, 0, 400, 300);
    vector<Point2f> centroids_color;
    detect_objects(img_color, img_ground_color, roi_color, 100, centroids_color);

    imshow(window_name_ir, img_ir);
    imshow(window_name_color, img_color);

    //Get location of the centroids with homography
    vector<Point2f> centroids_color_rw;
    cv::perspectiveTransform(centroids_color, centroids_color_rw, H_rgb.inv());

    vector<Point2f> centroids_ir_rw;
    cv::perspectiveTransform(centroids_ir, centroids_ir_rw, H_ir.inv());

    //Solution to the first exercise
    std::cout << "REAL WORLD CENTROIDS COLOR" << std::endl;
    std::cout << centroids_color_rw << std::endl;
    std::cout << "REAL WORLD CENTROIDS IR" << std::endl;
    std::cout << centroids_ir_rw << std::endl;

    cv::waitKey();
    
    /* Example of using Eigen to solve SVD
    MatrixXf m = MatrixXf::Random(3,2);
    cout << "Here is the matrix m:" << endl << m << endl;
    JacobiSVD<MatrixXf> svd(m, ComputeThinU | ComputeThinV);
    cout << "Its singular values are:" << endl << svd.singularValues() << endl;
    cout << "Its left singular vectors are the columns of the thin U matrix:" << endl << svd.matrixU() << endl;
    cout << "Its right singular vectors are the columns of the thin V matrix:" << endl << svd.matrixV() << endl;
    Vector3f rhs(1, 0, 0);
    cout << "Now consider this rhs vector:" << endl << rhs << endl;
    cout << "A least-squares solution of m*x = rhs is:" << endl << svd.solve(rhs) << endl;
    */
      
    return 0;
}