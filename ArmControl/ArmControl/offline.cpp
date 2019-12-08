#include <iostream>
#include <stdlib.h>

#include "perspective_transformation.cpp"
#include "opencv2/opencv.hpp"
#include "img_processing.cpp"

#define CAM_X 30.747501
#define CAM_Y 582.65411
#define CAM_Z 372.46118  // we have a gripper lenght of 4cm => subtract 40 mm from position

#define CAM_TH 0.0 // x
#define CAM_PSI 0.0 //y
#define CAM_PHI 180.0 //z

#define K_X 0.55555

using namespace std;
using namespace cv;

/**
 * super ugly brute force way in order to test varius configurations
 */
void test_p_transform() {
    workspace ws;
    prepare_workspace(CAM_X, CAM_Y, CAM_Z, CAM_TH, CAM_PSI, CAM_PHI, &ws);
    auto position = perform_perpective_transformation(make_tuple(K_X * (770 - 640.0), K_X * (345 - 480.0)), 1400.0,
                                                      ws);
    cout << get<0>(position) << " " << get<1>(position) << " " << get<2>(position) << endl;

}

int main(int argc, char **argv) {
    cout << "LEts go!" << endl;
    test_p_transform();
    return 0;
    // Get camera frame
    VideoCapture camera = init_camera(0);
    Mat frame = get_camera_frame(camera);
    imshow("Original camera frame", frame); // debug
    char c = (char) waitKey();

    // Set camera distortion parameters
    double f_x = 1208.0, f_y = 1212.0, c_x = 798.0, c_y = 595.4;
    double k_1 = 0.1013, k_2 = -0.0944, k_3 = -0.1801;
    double p_1 = 0, p_2 = 0;
    Mat cameraMat = (Mat_<double>(3, 3) << f_x, 0, f_y, 0, c_x, c_y, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

    // Undistort it
    Mat undistorted_frame = undistort_camera_frame(frame, cameraMat, distCoeffs);
    imshow("Undistorted camera frame", undistorted_frame); // debug
    c = (char) waitKey();

    // Get centroids
    Mat work_image = undistorted_frame.clone();
    // TODO check the channels and either convert the image or not
    cv::cvtColor(work_image, work_image, cv::COLOR_BGR2GRAY);

    imshow("Original camera frame 1", work_image); // debug
    waitKey();

    vector<tuple<double, double, double>> centroids;
    get_centroids(work_image, &centroids);

    cout << "Resulting centroid(s), principle angle(s):" << endl;
    for (tuple<double, double, double> ctr: centroids) {
        cout << get<0>(ctr) << " " << get<1>(ctr) << " " << get<2>(ctr) << " " << endl;
    }

    return 0;
}


