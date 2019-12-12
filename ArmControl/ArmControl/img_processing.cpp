#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;



Mat get_camera_frame(VideoCapture camera) {
    Mat view;
    camera >> view;
    return view;
}

void undistort_camera_frame(Mat *frame, Mat *undistorted, Matx33f *cameraMat, Mat *distCoeffs) {
    undistort(*frame, *undistorted, *cameraMat, *distCoeffs);
}

bool measure_ground_squares(VideoCapture &camera, double *square_px, int square_size, Size board_size) {
    const int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK;
    int successfull_measures = 0;
    int repeats = 5;
    double averages = 0;

    for (int i = 0; i < repeats; ++i) {
        Mat frame = get_camera_frame(camera);
        cout << "Measuring (" << i << "/5)...";
        vector<Point2f> pointBuf;

        bool found = cv::findChessboardCorners(frame, board_size, pointBuf, chessBoardFlags);

        if (found) {
            successfull_measures++;
            double sum = 0;
            for (int j = 0; j < board_size.height; ++j) {
                for (int k = 0; k < board_size.width - 1; ++k) {
                    int idx = board_size.width * j + k;
                    sum += pointBuf[idx + 1].x - pointBuf[idx].x;
                }
            }
            double avg = sum / ((board_size.width - 1) * board_size.height);
            averages += avg;

            cout << " \t\t => " << square_size << "mm is ~ " << avg << " px." << endl;
        } else {
            cerr << "Failed to find chessboard." << endl;
        }
    }

    *square_px = averages / repeats;
    return successfull_measures > 0;
}