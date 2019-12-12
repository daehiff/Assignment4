//
// Created by David Winderl on 2019-12-11.
//

#ifndef CAM_HELPER
#define CAM_HELPER

#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>
#include <fstream>

#include "opencv2/opencv.hpp"


using namespace std;
using namespace cv;


typedef struct workspace {
    Matx33f r_mat;
    Matx31f t_vec;
    cv::Matx33f camera_matrix;
    cv::Mat dist_coeffs;
} workspace;

/**
 * Get camera by its id
 * @param cameraID
 * @return
 */
cv::VideoCapture init_camera(int cameraID) {
    cv::VideoCapture inputCapture;
    inputCapture.open(cameraID);
    return inputCapture;
}


/**
 * Get the cameras intrinsic parameters from:
 * camparams.txt
 * Get the setup points from
 * setupPoints.txt
 * Use them in order to define a workspace, which holds:
 * the cameras position in space: (x, y, z, th, phi, psi)
 * the cameras distortion coefficients and the cameras matrix
 * @param ws
 * @return
 */
int prepare_workspace(workspace *ws) {
    vector<cv::Vec2f> image_points = {};
    vector<cv::Vec3f> object_points = {};

    cout << "Reading camera parameters" << endl;
    ifstream paramsFile;
    paramsFile.open("./camparams.txt");
    if (!paramsFile) {
        cerr << "camparams.txt file is missing.";
        return 1;
    }
    double f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3;
    paramsFile >> f_x >> f_y >> c_x >> c_y >> k_1 >> k_2 >> p_1 >> p_2 >> k_3;
    cv::Matx33f camera_matrix(f_x, 0, c_x,
                              0, f_y, c_y,
                              0, 0, 1);
    Mat dist_coeffs = (Mat_<double>(5, 1) << k_1, k_2, p_1, p_2, k_3);

    cout << "Computing Pnp" << endl;
    ifstream calibrationFile;
    calibrationFile.open("./setupPoints.txt");
    if (!paramsFile) {
        cerr << "setupPoints.txt file is missing. Please provide or calibrate";
        return 1;
    }
    string line;
    while (getline(calibrationFile, line)) {
        istringstream iss(line);
        double px_x, px_y, x, y, z;
        if (!(iss >> px_x >> px_y >> x >> y >> z)) { break; } // error
        image_points.emplace_back(Vec2f(px_x, px_y));
        object_points.emplace_back(Vec3f(x, y, z));
    }
    Matx31f r_vec;
    Matx33f r_mat;

    Matx31f t_vec;
    solvePnP(object_points, image_points, camera_matrix, dist_coeffs, r_vec, t_vec);
    Rodrigues(r_vec, r_mat);
    ws->t_vec = t_vec;
    ws->r_mat = r_mat;
    ws->dist_coeffs = dist_coeffs;
    ws->camera_matrix = camera_matrix;
    return 0;

}

#endif