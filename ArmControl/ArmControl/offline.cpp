#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include "opencv2/opencv.hpp"
#include "img_processing.cpp"
#include "get_centroids.cpp"

using namespace std;
using namespace cv;

#define REPEAT true
#define CAMERA_FLIP true

struct Vec2 {
    double x;
    double y;
    Vec2(double new_x, double new_y) : x(new_x), y(new_y) { }
};

Vec2 transform_to_world_coords(Vec2 camera_coords, Vec2 scaling_factor, Vec2 camera_orig_wrld) {
    return Vec2(
        camera_coords.x * scaling_factor.x + camera_orig_wrld.x,
        camera_coords.y * scaling_factor.y + camera_orig_wrld.y
    );
}

int main(int argc, char **argv) {
    if (argc < 4) {
        cerr << "usage: hw4 <camera_id> <orig_x> <orig_y> [ <scaling> ]";
        return 1;
    }

    Vec2 camera_wrld_orig = Vec2(atof(argv[2]), atof(argv[3]));

    // Read camera parameters from file
    cout << "Reading camera parameters" << endl;
    ifstream paramsFile;
    paramsFile.open("camparams.txt");
    if (!paramsFile) { cerr << "camparams.txt file is missing."; return 1; }
    double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;
    paramsFile >> f_x >> f_y >> c_x >> c_y >> k_1 >> k_2 >> p_1 >> p_2 >> k_3;

    // Connect to camera
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(atoi(argv[1]));
    waitKey(1000);

    // Get camera height
    double sf_x, sf_y;
    if (argc == 5) {
        sf_x = atof(argv[4]);
        sf_y = -sf_x;
    } else {
        double square_px;
        const int square_size = 5;
        bool measured = measure_ground_squares(camera, &square_px, square_size, Size(8,6));

        if (measured) {
            cout << square_size << "mm is ca. " << square_px << " pixels in camera space." << endl;
        } else {
            cerr << "Failed to measure the ground scaling." << endl;
            return 2;
        }

        sf_y = square_size / square_px;
        sf_x = -sf_y;
    }
    cout << "Scaling factor is [" << sf_x << ", " << sf_y << "]" << endl;
    cout << "Assuming the camera origin [0, 0] is at [" <<
        camera_wrld_orig.x << ", " << camera_wrld_orig.y << "] in robot space." << endl;

    do {
        Mat frame = get_camera_frame(camera);

        // Set camera distortion parameters
        Mat cameraMat = (Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);
        Mat distCoeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

        // Undistort it
        Mat undistorted_frame = undistort_camera_frame(frame, cameraMat, distCoeffs);
        cvtColor(undistorted_frame, undistorted_frame, COLOR_BGR2GRAY);
        imshow("Pre-processed camera frame", undistorted_frame); 
        waitKey(500);

        // Get centroids
        Mat work_image = undistorted_frame.clone();
        vector<tuple<double, double, double>> centroids;
        get_centroids(work_image, &centroids);
        cvtColor(work_image, work_image, COLOR_GRAY2RGB);

        for (tuple<double, double, double> ctr: centroids) {
            double x = get<0>(ctr), y = get<1>(ctr), angle = get<2>(ctr);
            double angle_rad = angle * 0.01745; // precision? pfft!
            cv::circle(work_image, Point(x, y), 5, Scalar(0, 0, 255), 4);
            cv::line(work_image, Point(x, y), Point(x + sin(angle_rad) * 20, y + cos(angle_rad) * 20), Scalar(255, 0, 255), 1);
            stringstream cc_str, wc_str;
            Vec2 robotCoords = transform_to_world_coords(Vec2(x,y), Vec2(sf_x, sf_y), camera_wrld_orig);
            cc_str << "[" << x << ", " << y << "]";
            wc_str << "[" << robotCoords.x << ", " << robotCoords.y << "]";
            cv::putText(work_image, cc_str.str().c_str(), Point(x + 15, y + 5), FONT_HERSHEY_SIMPLEX, 0.33, Scalar(0, 0, 255), 1);
            cv::putText(work_image, wc_str.str().c_str(), Point(x + 15, y + 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 128, 255), 2);
        }

        imshow("Results", work_image);
        waitKey(1000);
    } while (REPEAT);

    return 0;
}
