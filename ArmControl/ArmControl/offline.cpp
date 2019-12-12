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

/**
 * Method to get the objects centroids from the workspace
 * @param camera
 * @param centroids
 * @param ws
 * @param debug
 */
void get_centroids_from_camera(VideoCapture *camera, vector<tuple<double, double, double>> *centroids,
                               workspace *ws = nullptr, Mat *result = nullptr) {
    Mat frame, work_image;
    *camera >> frame;
    if (ws != nullptr) {
        cv::undistort(frame, work_image, ws->camera_matrix, ws->dist_coeffs);
    } else {
        work_image = frame.clone();
    }
    cvtColor(work_image, work_image, COLOR_RGB2GRAY);
    get_centroids(&work_image, centroids);
    if (result != nullptr) {
        cvtColor(work_image, work_image, COLOR_GRAY2RGB);
        *result = work_image;
    }
}

/**
 * Routine with which we can calibrate the camera
 * @param argc
 * @param argv
 * @return
 */
int camera_calibration(int argc, char **argv) {
    workspace ws;
    int cam_id = -1;
    if (argc == 2) {
        cam_id = atoi(argv[1]);
        if (cam_id == -1) {
            cerr << "Invalid Camera Id";
            return 1;
        }
    } else {
        cerr << "Please run with: <camera_id>";
        return 1;
    }
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(cam_id);
    vector<cv::Vec2f> centroids;
    vector<cv::Vec3f> real_world_coordinates;
    do {
        while (true) {
            cout << "Confirm Workspace Setup: " << endl;
            string line;
            getline(std::cin, line);
            if (line == "y" || line == "Y") {
                break;
            }
        }
        vector<tuple<double, double, double>> tmp_centroids;
        Mat result;
        get_centroids_from_camera(&camera, &tmp_centroids, nullptr, &result);
        for (auto ctr: tmp_centroids) {
            double px_x = get<0>(ctr);
            double px_y = get<1>(ctr);
            double angle = get<2>(ctr);
            double x = -1, y = -1, z = -1;
            Mat work_image = result.clone();
            cout << "At Centroid: " << px_x << ", " << px_y << endl;
            cv::circle(work_image, Point2f(px_x, px_y), 5, Scalar(0, 0, 255), 4);
            cv::line(work_image, Point2f(px_x, px_y), Point(px_x + sin(angle) * 20, px_y + cos(angle) * 20),
                     Scalar(255, 0, 255), 1);
            stringstream cc_str, wc_str;
            cv::putText(work_image, cc_str.str().c_str(),
                        Point2f(px_x + 15, px_y + 5),
                        FONT_HERSHEY_SIMPLEX, 0.33,
                        Scalar(0, 0, 255), 1);
            cv::putText(work_image, wc_str.str().c_str(),
                        Point2f(px_x + 15, px_y + 25),
                        FONT_HERSHEY_SIMPLEX, 0.5,
                        Scalar(0, 128, 255), 2);
            cout << "Look at the image and move the robot arm to the object then insert the real world coordinates"
                 << endl;
            imshow("Output", work_image);
            waitKey();
            while (true) {
                cout << "Please insert real world coordinates as the following: x y z" << endl;
                string line;
                getline(std::cin, line);
                stringstream ss(line);
                ss >> x >> y >> z;
                cout << "Confirm: " << x << " " << y << " " << z << " ";
                getline(std::cin, line);
                if (!(line == "y" || line == "Y")) {
                    x = -1;
                    y = -1;
                    z = -1;
                } else {
                    break;
                }
            }
            centroids.emplace_back(px_x, px_y);
            real_world_coordinates.emplace_back(x, y, z);
        }
        cout << "Done! No more centroids found!" << endl;
        cout << endl;
    } while (centroids.size() <= 12);

    ofstream paramsFile;
    paramsFile.open("./setupPoints.txt");
    if (!paramsFile) {
        cerr << "setupPoints.txt file is missing.";
        return 1;
    }
    for (int i = 0; i < centroids.size(); ++i) {
        paramsFile << "" << centroids[i](0) << " " << centroids[i](1) << "=>"
                   << real_world_coordinates[i](0) << " "
                   << real_world_coordinates[i](1) << " "
                   << real_world_coordinates[i](0)
                   << endl;
    }
    return 0;
}


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


