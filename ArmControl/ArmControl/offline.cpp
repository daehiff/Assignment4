#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "opencv2/opencv.hpp"
#include "img_processing.cpp"
#include "get_centroids.cpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "usage: hw4 <camera_id>";
        return 1;
    }

    // Read camera parameters from file
    cout << "Reading camera parameters" << endl;
    ifstream paramsFile;
    paramsFile.open("camparams.txt");
    if (!paramsFile) { cerr << "camparams.txt file is missing."; return 1; }
    double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;
    paramsFile >> f_x >> f_y >> c_x >> c_y >> k_1 >> k_2 >> p_1 >> p_2 >> k_3;

    // Get camera frame
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(atoi(argv[1]));
    waitKey(1000);

    while (true) {
        Mat frame = get_camera_frame(camera);

        // Set camera distortion parameters
        Mat cameraMat = (Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);
        Mat distCoeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

        // Undistort it
        Mat undistorted_frame = undistort_camera_frame(frame, cameraMat, distCoeffs);
        cvtColor(undistorted_frame, undistorted_frame, COLOR_BGR2GRAY);
        imshow("Pre-processed camera frame", undistorted_frame); 

        // Get centroids
        Mat work_image = undistorted_frame.clone();
        vector<tuple<double, double, double>> centroids;
        get_centroids(work_image, &centroids);
        cvtColor(work_image, work_image, COLOR_GRAY2RGB);

        cout << "Resulting centroid(s), principle angle(s):" << endl;
        for (tuple<double, double, double> ctr: centroids) {
            double x = get<0>(ctr), y = get<1>(ctr), angle = get<2>(ctr);
            double angle_rad = angle * 0.01745; // precision? pfft!
            cout << "[" << x << ", " << y << ", " << angle << "]" << endl;
            cv::circle(work_image, Point(x, y), 5, Scalar(0, 0, 255), 4);
            cv::line(work_image, Point(x, y), Point(x + sin(angle_rad) * 20, y + cos(angle_rad) * 20), Scalar(255, 0, 255), 1);
        }

        imshow("Results", work_image);
        waitKey(1000);
    }

    return 0;
}
