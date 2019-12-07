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


#define EPSILON 0.01
#define PI 3.141592653589793238463


typedef struct workspace {
    double d_x;
    double d_y;
    double d_z;
    double r_th;  /* rotation around the x-axis (deg)*/
    double r_psi; /* rotation around the y-axis (deg)*/
    double r_phi; /*rotation around the z-axis (deg)*/
} workspace;


using namespace std;
using Eigen::MatrixXd;

int get_transformation_matrix(MatrixXd *trans_m, workspace ws) {
    MatrixXd m_x(4, 4);
    m_x << 1, 0, 0, 0,
            0, cos(ws.r_th), -sin(ws.r_th), 0,
            0, sin(ws.r_th), cos(ws.r_th), 0,
            0, 0, 0, 1;
    MatrixXd m_y(4, 4);
    m_y << cos(ws.r_psi), 0, -sin(ws.r_psi), 0,
            0, 1, 0, 0,
            sin(ws.r_psi), 0, cos(ws.r_psi), 0,
            0, 0, 0, 1;
    MatrixXd m_z(4, 4);
    m_z << cos(ws.r_phi), -sin(ws.r_phi), 0, 0,
            sin(ws.r_phi), cos(ws.r_phi), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    MatrixXd trans(4, 4);
    trans << 1, 0, 0, ws.d_x,
            0, 1, 0, ws.d_y,
            0, 0, 1, ws.d_z,
            0, 0, 0, 1;
    MatrixXd rpy(4, 4);
    rpy = m_x * m_y * m_z;
    *trans_m = m_x * m_y * m_z * trans;
    return 0;
}

void prepare_workspace(double x, double y, double z,
                       double th, double psi, double phi,
                       workspace *ws) {
    ws->d_x = x;
    ws->d_y = y;
    ws->d_z = z;
    ws->r_th = (th * PI / 180.0);
    ws->r_psi = (psi * PI / 180.0);
    ws->r_phi = (phi * PI / 180.0);

}

/**
 * performs a perspective transformation of an image point towards a point in the coords of the robot according to its workspace
 * @param point_image
 * @param f
 * @param ws
 * @return
 */
tuple<double, double, double> perform_perpective_transformation(tuple<double, double> point_image, double f,
                                                                workspace ws) {
    double p_i1 = get<0>(point_image);
    double p_i2 = get<1>(point_image);
    double lambda = -ws.d_z;
    MatrixXd p_c(4, 1);
    MatrixXd tmp(4, 1);
    MatrixXd t_cam_base(4, 4);
    MatrixXd p_b_c;
    p_c << (p_i1 / f * (f - lambda)),
            (p_i2 / f * (f - lambda)),
            (lambda),
            1;
    assert(abs(-(f * p_c(0, 0)) / (p_c(2, 0) - f) - p_i1) < EPSILON); // retransformation for every coord must be fine
    assert(abs(-(f * p_c(1, 0)) / (p_c(2, 0) - f) - p_i2) < EPSILON);
    get_transformation_matrix(&t_cam_base, ws);
    p_b_c = t_cam_base * p_c;
    tmp = t_cam_base.inverse() * p_b_c;
    for (int i = 0; i < 4; ++i) {
        assert(abs(p_c(i, 0) - tmp(i, 0)) < EPSILON); // check if
    }
    return make_tuple(p_b_c(0, 0), p_b_c(1, 0), p_b_c(2, 0));
}
