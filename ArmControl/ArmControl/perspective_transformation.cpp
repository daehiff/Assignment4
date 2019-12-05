// This program is a simple template of an C++ program loading and showing image with OpenCV.
// You can ignore this file and write your own program.
// The program takes a image file name as an argument.

#include <stdio.h>
#include <unordered_map>
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"
#include <opencv2/imgproc/imgproc.hpp>
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>


#define PI 3.141592653589793238463


#define D_X 0                         /* translation along the x axis*/
#define D_Y 0                         /* translation along the y axis*/
#define D_Z 0                         /* translation along the z axis*/


#define TH 0                          /* rotation around the x-axis (deg)*/
#define PSI 0                          /* rotation around the y-axis (deg)*/
#define PHI 0                          /*rotation around the z-axis (deg)*/


#define R_TH (TH * PI / 180.0)         /* rotation around the x-axis (rad)*/
#define R_PSI (PSI * PI / 180.0)       /* rotation around the y-axis (rad)*/
#define R_PHI (PHI * PI / 180.0)       /*rotation around the z-axis (rad)*/


using namespace std;
using Eigen::MatrixXd;

int get_rpy_matrix(MatrixXd *trans_m) {
    cout << R_TH << endl;
    MatrixXd m_x(4, 4);
    m_x << 1, 0, 0, 0,
            0, cos(R_TH), -sin(R_TH), 0,
            0, sin(R_TH), cos(R_TH), 0,
            0, 0, 0, 1;
    MatrixXd m_y(4, 4);
    m_y << cos(R_PSI), 0, sin(R_PSI), 0,
            0, 1, 0, 0,
            -sin(R_PSI), 0, cos(R_PSI), 0,
            0, 0, 0, 1;
    MatrixXd m_z(4, 4);
    m_z << cos(R_PHI), sin(R_PHI), 0, 0,
            sin(R_PHI), cos(R_PHI), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    MatrixXd trans(4, 4);
    trans << 1, 0, 0, D_X,
            0, 1, 0, D_Y,
            0, 0, 1, D_Z,
            0, 0, 0, 1;
    MatrixXd rpy(4, 4);
    rpy = m_x * m_y * m_z;
    *trans_m = rpy * trans;
    return 0;
}

/**
 *
 * @param point_image
 * @param f
 * @return
 */
tuple<double, double, double> perform_perpective_transformation(tuple<double, double> point_image, double f) {
    double p_i1 = get<0>(point_image);
    double p_i2 = get<1>(point_image);
    double lambda = D_Z; // TODO validate this
    MatrixXd p_c(4, 1);
    MatrixXd transformation(4, 4);
    MatrixXd out;
    p_c << (p_i1 / f * (f - lambda)),
            (p_i2 / f * (f - lambda)),
            (lambda),
            1;
    cout << "PC:" << endl;
    cout << p_c << endl;
    get_rpy_matrix(&transformation);
    out = transformation * p_c;
    cout << "Out:" << endl;
    cout << out << endl;
    return make_tuple(out(0, 0), out(1, 0), out(2, 0));
}
