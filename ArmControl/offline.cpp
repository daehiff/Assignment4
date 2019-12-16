#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

#include "opencv2/opencv.hpp"
#include "img_processing.cpp"
#include "get_centroids.cpp"

#define is_windows 0


using namespace std;
using namespace cv;

#define REPEAT true

struct Pose {
    int x;
    int y;
    int z;
    int degX;
    int degY; 
    int degZ;

    Pose(int x, int y, int z, int degX, int degY, int degZ): x(x), y(y), z(z), degX(degX), degY(degY), degZ(degZ) { };
    static Pose from_centroid(int z, int degX) {
        return Pose(0, 0, z, degX, 0, 0);
    }
};


int whatever() {
    int camid = 0;
    VideoCapture camera = init_camera(camid);
    Mat frame = get_camera_frame(camera);
    Mat work_image = frame.clone();
    cvtColor(work_image, work_image, COLOR_BGR2GRAY);

    // Get centroids
    vector<tuple<double, double, double>> centroids;
    get_centroids(work_image, &centroids);
    return 0;


}

int main(int argc, char **argv) {
    return whatever();
    if (argc < 2) {
        cerr << "usage: hw4 <camera_id> [ <scaling> ]";
        return 1;
    }

    // Read camera parameters from file
    cout << "Reading camera parameters..." << endl;
    ifstream paramsFile;
    paramsFile.open("camparams.txt");
    if (!paramsFile) { cerr << "camparams.txt file is missing."; return 1; }
    double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;
    paramsFile >> f_x >> f_y >> c_x >> c_y >> k_1 >> k_2 >> p_1 >> p_2 >> k_3;

    // Set camera distortion parameters
    Mat cameraMat = (Mat_<double>(3, 3) << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1);
    Mat distCoeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

    // Connect to camera
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(atoi(argv[1]));
    waitKey(2000);
    cout << "Connected." << endl;

    // Get scaling
    Vec2 scaling = (argc >= 3) ? Vec2(-atof(argv[2]),atof(argv[2])) : measure_scaling(&camera);
    cout << "Scaling factor is [" << scaling.x << ", " << scaling.y << "]" << endl;

    // Read known points
    int known_points_count;
    Vec2 *known_points_cam, *known_points_world;
    read_known_points(&known_points_count, known_points_cam, known_points_world);

    Vec2 camera_orig_world = estimate_origin(known_points_cam, known_points_world,
        known_points_count, scaling);

    cout << "Assuming the camera origin [0, 0] is at [" <<
        camera_orig_world.x << ", " << camera_orig_world.y << "] in robot space." << endl;

    perform_sanity_check(known_points_count, known_points_cam, known_points_world, scaling, camera_orig_world);

    do {
        Mat frame = get_camera_frame(camera);

        // Undistort & convert the image
        Mat work_image = undistort_camera_frame(frame, cameraMat, distCoeffs);
        cvtColor(work_image, work_image, COLOR_BGR2GRAY);

        // Get centroids
        vector<tuple<double, double, double>> centroids;
        get_centroids(work_image, &centroids);

        // Display debug information on the screen
        cvtColor(work_image, work_image, COLOR_GRAY2RGB);
        display_points(work_image, centroids, scaling, camera_orig_world);
        imshow("Results", work_image);
        waitKey(500);
    } while (REPEAT);

    return 0;
}
