#include <iostream>
#include <stdlib.h>
#include <iomanip>

#include "perspective_transformation.cpp"
#include "opencv2/opencv.hpp"
#include "img_processing.cpp"

#define CAM_X 30.747501
#define CAM_Y 582.65411
#define CAM_Z 372.46118

#define CAM_TH 0.0 // x
#define CAM_PSI 0.0 //y
#define CAM_PHI 0.0 //z

#define F_X 1.4002144943143016e+03
#define F_Y 1.4031326269219808e+03
#define C_X 6.4521582732650791e+02
#define C_Y 4.6378737841255867e+02


using namespace std;
using namespace cv;

/**
 * super ugly brute force way in order to test varius configurations
 */
void test_p_transform() {
    workspace ws;
    cv::Matx33f camera_matrix(F_X, 0, C_X,
                              0, F_Y, C_Y,
                              0, 0, 1); // TODO @Marian do we should have it already and not hardcoded in here
    prepare_workspace(CAM_X, CAM_Y, CAM_Z, CAM_TH, CAM_PSI, CAM_PHI, camera_matrix, &ws);
    vector<vector<double>> image_vals = {
            {0,       0},
            {770,     345},
            {422.431, 634.475},
            {877.135, 625.746},
            {643.138, 483.524},
    };
    vector<vector<double>> measured_output = {
            {-342.222222, 271.6666667, -229}, // TODO @Marian -342 == 342?
            {-54,         464,         -215},
            {120,         589,         -220},
            {-98,         594,         -212},
            {15,          540,         -215},
    };
    // 1280.0, 345.0 / 960.0
    for (int i = 0; i < image_vals.size(); ++i) {
        cv::Matx31f position;
        cv::Matx21f image_point(image_vals[i][0], image_vals[i][1]);
        perform_perpective_transformation(&image_point, &position, &ws);
        cout << image_vals[i][0] << ", " << image_vals[i][1] << ": "
             << position(0, 0) << " " << position(0, 1) << " " << position(0, 2)
             << endl;
        cout << image_vals[i][0] << ", " << image_vals[i][1] << ": "
             << (int) measured_output[i][0] << " " << (int) measured_output[i][1] << " " << (int) measured_output[i][2]
             << endl;
        cout << endl;
    }

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


