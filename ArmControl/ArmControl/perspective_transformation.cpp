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
 * convert a raw image point into 3D world coordinates in the cameras coordinate system
 * @param image_point
 * @param world_cord
 */
void perform_perspective_transformation(cv::Vec2f *image_point, cv::Vec3f *world_cord, workspace *ws,
                                        double z = 232.0) {
    double x_im = (*image_point)(1);
    double y_im = (*image_point)(0);
    double scaling_factor;
    Matx31f tmp, tmp1, uv_point;
    Matx33f i_rmat, i_cammat;

    i_rmat = ws->r_mat.inv();
    i_cammat = ws->camera_matrix.inv();
    uv_point << y_im, x_im, 1;
    tmp = (i_rmat * i_cammat) * uv_point;
    tmp1 = i_rmat * ws->t_vec;
    scaling_factor = z + tmp1(2, 0) / tmp(2, 0);
    tmp = i_rmat * ((scaling_factor * i_cammat * uv_point) - ws->t_vec);
    *world_cord << tmp(0, 0), tmp(0, 1), tmp(0, 2);
}


