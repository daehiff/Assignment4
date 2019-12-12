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
#include "camera_helper.cpp"


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
 * convert a raw image point into 3D world coordinates in the cameras coordinate system
 * @param image_point
 * @param world_cord
 */
void convert_to_pixels(cv::Vec2f *image_point, cv::Vec3f *world_cord, workspace *ws) {
    double scaling_factor = -(372.46118 + 229.69);
    double x_im = (*image_point)(0);
    double y_im = (*image_point)(1);
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
void perform_perspective_transformation(cv::Vec2f *image_point, cv::Vec3f *robot_cords, workspace *ws) {
    cv::Matx44f trans_m;
    get_transformation_matrix(&trans_m, ws);
    cv::Vec4f p_b_c; // point respective from base to world coordinate
    cv::Vec3f world_cord; // point of the image in world coordinates
    convert_to_pixels(image_point, &world_cord, ws);
    p_b_c << (world_cord(0)), (world_cord(1)), world_cord(2), 1;
    p_b_c = trans_m * p_b_c;
    *robot_cords << p_b_c(0), p_b_c(1), p_b_c(2);
}

/**
 * @Marians perspective transformation
 * @param image_point
 * @param world_point
 * @param ws
 * @param sf_x
 * @param sf_y
 */
void perform_perpective_transformation_1(cv::Vec2f *image_point, cv::Vec3f *world_point, workspace *ws,
                                         double sf_x, double sf_y) {
    *world_point << (*image_point)(0) * sf_x + ws->d_x,
            (*image_point)(1) * sf_y + ws->d_y,
            ws->d_z;
}

