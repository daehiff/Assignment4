#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

#define BOTTOM_Z -225
#define SQUARE_SIZE_MM 25

struct Vec2 {
    double x;
    double y;
    Vec2(double new_x, double new_y) : x(new_x), y(new_y) { }
    Vec2() : x(0), y(0) { }
};

void read_known_points(int * known_points_count, Vec2 * known_points_cam, Vec2 * known_points_world) {
    // Reading known points
    cout << "Reading known points..." << endl;
    ifstream pointsFile;
    pointsFile.open("knownpoints.txt");
    if (!pointsFile) { cerr << "knownpoints.txt file is missing."; return; }
    pointsFile >> *known_points_count;
    known_points_cam = new Vec2[*known_points_count];
    known_points_world = new Vec2[*known_points_count];
    for (int i = 0; i < *known_points_count; i++) {
        pointsFile >> known_points_cam[i].x >> known_points_cam[i].y;
        pointsFile >> known_points_world[i].x >> known_points_world[i].y;
    }
}

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

void undistort_camera_frame(Mat *frame, Mat *undistorted, Matx33f *cameraMat, Mat *distCoeffs) {
    undistort(*frame, *undistorted, *cameraMat, *distCoeffs);
}

Vec2 transform_to_world_coords(Vec2 camera_coords, Vec2 scaling_factor, Vec2 camera_orig_wrld) {
    return Vec2(
        camera_coords.x * scaling_factor.x + camera_orig_wrld.x,
        camera_coords.y * scaling_factor.y + camera_orig_wrld.y
    );
}

bool measure_ground_squares(VideoCapture & camera, double * square_px, int square_size, Size board_size) {
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

Vec2 measure_scaling(VideoCapture * camera) {
    double sf_x, sf_y;
    double square_px;
    const int square_size = SQUARE_SIZE_MM;
    bool measured = measure_ground_squares(*camera, &square_px, square_size, Size(8,6));

    if (measured) {
        cout << square_size << "mm is ca. " << square_px << " pixels in camera space." << endl;
    } else {
        cerr << "Failed to measure the ground scaling." << endl;
        return Vec2(-1, -1);
    }

    sf_y = square_size / square_px;
    sf_x = -sf_y;
    
    return Vec2(sf_x, sf_y);
}

void perform_sanity_check(int known_points_count, Vec2 * known_points_cam, Vec2 * known_points_world, Vec2 scaling, Vec2 camera_orig_world) {
    cout << "Sanity checks: " << endl;
    for (int i = 0; i < known_points_count; i++) {
        Vec2 world_coords = transform_to_world_coords(Vec2(known_points_cam[i].x,known_points_cam[i].y), scaling, camera_orig_world);
        cout << " - (" << known_points_cam[i].x << ", " << known_points_cam[i].y << ")\t->\t("
            << world_coords.x << ", " << world_coords.y << ", " << BOTTOM_Z << ")"
            << "\t\t expected to be (" << known_points_world[i].x << ", " << known_points_world[i].y 
            << ", ~" << BOTTOM_Z << ")" << endl;
    }
}

Vec2 estimate_origin(Vec2 * known_points_cam, Vec2 * known_points_world, int known_points, Vec2 scaling) {
    double total_x = 0, total_y = 0;

    for (int i = 0; i < known_points; i++) {
        total_x += known_points_world[i].x - known_points_cam[i].x * scaling.x;
        total_y += known_points_world[i].y - known_points_cam[i].y * scaling.y;
        cout << known_points_world[i].x - known_points_cam[i].x * scaling.x
        << "\t" << known_points_world[i].y - known_points_cam[i].y * scaling.y << endl;
    }

    return Vec2(total_x / known_points, total_y / known_points);
}

/** renders those debug graphics and texts into the image **/
void display_points(Mat image_frame, vector<tuple<double, double, double>> & centroids, Vec2 scaling, Vec2 camera_orig_world) {
    int i = 0;
    for (tuple<double, double, double> ctr: centroids) {
        
        double x = get<0>(ctr), y = get<1>(ctr), angle = get<2>(ctr);
        double angle_rad = angle * 0.01745; // precision? pfft!
        cv::circle(image_frame, Point(x, y), 5, Scalar(0, 0, 255), 4);
        cv::line(image_frame, Point(x, y), Point(x + sin(angle_rad) * 20, y + cos(angle_rad) * 20), Scalar(255, 0, 255), 1);
        stringstream cc_str, wc_str, th_str, id_str;
        Vec2 robotCoords = transform_to_world_coords(Vec2(x,y), scaling, camera_orig_world);
        cc_str << "[" << x << ", " << y << "]";
        wc_str << "[" << robotCoords.x << ", " << robotCoords.y << "]";
        th_str << angle << " deg";
        id_str << "ID " << i;
        // display texts
        cv::putText(image_frame, id_str.str().c_str(), Point(x + 17, y - 15), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 0), 4);
        cv::putText(image_frame, id_str.str().c_str(), Point(x + 15, y - 15), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255), 2);

        cv::putText(image_frame, cc_str.str().c_str(), Point(x + 15, y + 5), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 0), 4);
        cv::putText(image_frame, cc_str.str().c_str(), Point(x + 15, y + 5), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1);

        cv::putText(image_frame, wc_str.str().c_str(), Point(x + 17, y + 32), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 4);
        cv::putText(image_frame, wc_str.str().c_str(), Point(x + 15, y + 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 128, 255), 2);

        cv::putText(image_frame, th_str.str().c_str(), Point(x + 17, y + 62), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 0, 0), 4);
        cv::putText(image_frame, th_str.str().c_str(), Point(x + 15, y + 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 255), 2);
        i++;
    }
}