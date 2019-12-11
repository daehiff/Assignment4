// This program is a simple template of an C++ program loading and showing image with OpenCV.
// You can ignore this file and write your own program.
// The program takes a image file name as an argument.

#include <stdio.h>
#include <unordered_map>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <assert.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>


#define EPSILON 0.001
#define PI 3.141592653589793238463


typedef struct workspace {
    double d_x;
    double d_y;
    double d_z;
    double r_th;  /* rotation around the x-axis (deg)*/
    double r_psi; /* rotation around the y-axis (deg)*/
    double r_phi; /*rotation around the z-axis (deg)*/
    double sf_x; // scaling factor?
    double sf_y;
    cv::Matx33f camera_matrix;
    cv::Mat dist_coeffs;
} workspace;


using namespace std;

/**
 * Get the spatial transformation matrix from camera to robot arm from the workspace
 * @param trans_m
 * @param ws
 * @return
 */
int get_transformation_matrix(cv::Matx44f *trans_m, workspace *ws) {
    cv::Matx44f m_x(4, 4);
    m_x << 1, 0, 0, 0,
            0, cos(ws->r_th), -sin(ws->r_th), 0,
            0, sin(ws->r_th), cos(ws->r_th), 0,
            0, 0, 0, 1;
    cv::Matx44f m_y(4, 4);
    m_y << cos(ws->r_psi), 0, sin(ws->r_psi), 0,
            0, 1, 0, 0,
            -sin(ws->r_psi), 0, cos(ws->r_psi), 0,
            0, 0, 0, 1;
    cv::Matx44f m_z(4, 4);
    m_z << cos(ws->r_phi), -sin(ws->r_phi), 0, 0,
            sin(ws->r_phi), cos(ws->r_phi), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    cv::Matx44f trans(4, 4);
    trans << 1, 0, 0, ws->d_x,
            0, 1, 0, ws->d_y,
            0, 0, 1, ws->d_z,
            0, 0, 0, 1;
    *trans_m = ((m_x * m_y) * m_z) * trans;
    return 0;
}

/**
 * Fill all necessary parameters into the workspace struct
 * @param x
 * @param y
 * @param z
 * @param th
 * @param psi
 * @param phi
 * @param ws
 */
void prepare_workspace(double x, double y, double z,
                       double th, double psi, double phi,
                       double sf_x, double sf_y,
                       cv::Matx33f camera_matrix, cv::Mat dist_coeffs,
                       workspace *ws) {
    ws->d_x = x;
    ws->d_y = y;
    ws->d_z = z;
    ws->sf_x = sf_x;
    ws->sf_y = sf_y;
    ws->r_th = (th * PI / 180.0);
    ws->r_psi = (psi * PI / 180.0);
    ws->r_phi = (phi * PI / 180.0);
    ws->camera_matrix = camera_matrix;
    ws->dist_coeffs = dist_coeffs;

}

/**
 * convert a raw image point into 3D world coordinates in the cameras coordinate system
 * @param image_point
 * @param world_cord
 */
void convert_to_pixels(cv::Matx21f *image_point, cv::Matx31f *world_cord, workspace *ws) {
    double scaling_factor = -(372.46118 + 229.69 + 5.0); // Assume the image is 1mm below the sensor
    double x_im = (*image_point)(0, 0);
    double y_im = (*image_point)(0, 1);
    *world_cord << x_im, y_im, 1;
    *world_cord = ws->camera_matrix.inv() * (*world_cord);
    *world_cord *= scaling_factor;
}

/**
 * convert a raw image point into a 3D point in robot arm coordinates
 * @param image_point
 * @param f
 * @param ws
 * @return
 */
void perform_perpective_transformation(cv::Matx21f *image_point, cv::Matx31f *robot_cords, workspace *ws) {
    cv::Matx44f trans_m;
    get_transformation_matrix(&trans_m, ws);
    cv::Matx41f p_b_c; // point respective from base to world coordinate
    cv::Matx31f world_cord; // point of the image in world coordinates
    convert_to_pixels(image_point, &world_cord, ws);
    p_b_c << (world_cord(0, 0)), (world_cord(0, 1)), world_cord(0, 2), 1;
    p_b_c = trans_m * p_b_c;
    *robot_cords << p_b_c(0, 0), p_b_c(0, 1), p_b_c(0, 2);
}


void perform_perpective_transformation_1(cv::Matx12f *image_point, cv::Matx13f *world_point, workspace *ws) {
    *world_point << (*image_point)(0, 0) * ws->sf_x + ws->d_x,
            (*image_point)(0, 1) * ws->sf_y + ws->d_y,
            ws->d_z;
}