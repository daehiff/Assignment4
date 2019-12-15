#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>
#include <fstream>

#include "perspective_transformation.cpp"
#include "opencv2/opencv.hpp"
#include "img_processing.cpp"
#include "get_centroids.cpp"
#include "camera_helper.cpp"

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


int main(int argc, char **argv) {
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

int main(int argc, char **argv) {
    //return camera_calibration(argc, argv);
    // getting parameters and prepare workspace
    workspace ws;
    double cam_id;
    VideoCapture camera;

    if (argc == 2) {
        cam_id = atoi(argv[1]);
        if (cam_id == -1) {
            cerr << "Invalid: <camera_id>";
            return 1;
        }
    } else {
        cerr << "please provide: <camera_id>";
        return 1;
    }
    cout << "Preparing Workspace" << endl;
    if (prepare_workspace(&ws)) {
        cerr << "Error preparing workspace";
        return 1;
    }
    cout << "Getting Camera" << endl;
    camera = init_camera(cam_id);
    vector<tuple<double, double, double>> centroids;

//    643.138 483.524 15.0 540.0 -215.0
//    0.0 0.0 -342.222222 271.6666667 -215.0
    vector<Vec2f> image_points = {
            Vec2f(643.138, 483.524),
            Vec2f(0.0, 0.0),
    };
    vector<Vec3f> world_points = {
            Vec3f(15.0, 540.0, -215.0),
            Vec3f(342.222222, 271.6666667, -215.0),
    };
    cout << ws.t_vec << endl;
    for (int i = 0; i < 2; ++i) {
        Vec3f room_vector;
        perform_perspective_transformation(&image_points[i], &room_vector, &ws);
        cout << room_vector << endl;
        cout << world_points[i] << endl;
    }
    return 0;
}


