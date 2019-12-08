#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

VideoCapture init_camera(int cameraID) {
    VideoCapture inputCapture;
    inputCapture.open(cameraID);
    return inputCapture;
}

Mat get_camera_frame(VideoCapture camera) {
    Mat view;
    camera >> view;
    return view;
}

Mat undistort_camera_frame(Mat frame, Mat cameraMat, Mat distCoeffs) {
    Mat undistorted;
    undistort(frame, undistorted, cameraMat, distCoeffs);
    return undistorted;
}