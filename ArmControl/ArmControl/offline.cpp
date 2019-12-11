#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <iomanip>

#include "perspective_transformation.cpp"
#include "opencv2/opencv.hpp"
#include "img_processing.cpp"
#include "get_centroids.cpp"

#define CAM_Z 372.46118 // offset of the camera

#define CAM_TH 0.0 // rotation around x
#define CAM_PSI 0.0 // rotation around y
#define CAM_PHI 0.0 // rotation around z


#define REPEAT false
#define CAMERA_FLIP true // TODO @Marian what is this can we delete it?


using namespace std;
using namespace cv;


/**
 * test the perspective transformation according to real world values
 */
void test_p_transform(workspace *ws) {
    vector<vector<double>> image_vals = {
            {0,       0},
            {770,     345},
            {422.431, 634.475},
            {877.135, 625.746},
            {643.138, 483.524},
    };
    vector<vector<double>> measured_output = {
            {-342.222222, 271.6666667, -229}, // TODO @Marian -342 == 342?
            {-54,         464,         -215},
            {120,         589,         -220},
            {-98,         594,         -212},
            {15,          540,         -215},
    };
    for (int i = 0; i < image_vals.size(); ++i) {
        cv::Matx31f position;
        cv::Matx21f image_point(image_vals[i][0], image_vals[i][1]);
        perform_perpective_transformation(&image_point, &position, ws);
        cout << image_vals[i][0] << ", " << image_vals[i][1] << ": "
             << position(0, 0) << " " << position(0, 1) << " " << position(0, 2)
             << endl;
        cout << image_vals[i][0] << ", " << image_vals[i][1] << ": "
             << (int) measured_output[i][0] << " " << (int) measured_output[i][1] << " " << (int) measured_output[i][2]
             << endl;
        cout << endl;
    }

}

/**
 * Test the centroid computation with the camera
 * @param argc
 * @param argv
 * @return
 */
void test_centroid_computation(VideoCapture camera, workspace ws) {
    do {
        Mat frame = get_camera_frame(camera);
        Mat undistorted;

        // Undistort it
        cv::undistort(frame, undistorted, ws.camera_matrix, ws.dist_coeffs);
        cvtColor(undistorted, undistorted, COLOR_BGR2GRAY);
        imshow("Pre-processed camera frame", undistorted);
        waitKey();

        // Get centroids
        Mat work_image = undistorted.clone();
        vector<tuple<double, double, double>> centroids;
        std::time_t t = std::time(0);  // t is an integer type
        get_centroids(&work_image, &centroids);
        std::cout << std::time(0) - t << " seconds\n";
        cvtColor(work_image, work_image, COLOR_GRAY2RGB);

        for (tuple<double, double, double> ctr: centroids) {
            double x = get<0>(ctr), y = get<1>(ctr), angle = get<2>(ctr);
            double angle_rad = angle; // precision? pfft! I converted it on deg, look into others code! :D
            cv::circle(work_image, Point(x, y), 5, Scalar(0, 0, 255), 4);
            cv::line(work_image, Point(x, y), Point(x + sin(angle_rad) * 20, y + cos(angle_rad) * 20),
                     Scalar(255, 0, 255), 1);
            stringstream cc_str, wc_str;
            Matx12f image_point(x, y);
            cv::Matx13f world_point;
            perform_perpective_transformation_1(&image_point, &world_point, &ws);
            cc_str << "[" << x << ", " << y << "]";
            wc_str << "[" << world_point(0, 0) << ", " << world_point(0, 1) << ", " << world_point(0, 2) << "]";
            cv::putText(work_image, cc_str.str().c_str(), Point(x + 15, y + 5), FONT_HERSHEY_SIMPLEX, 0.33,
                        Scalar(0, 0, 255), 1);
            cv::putText(work_image, wc_str.str().c_str(), Point(x + 15, y + 25), FONT_HERSHEY_SIMPLEX, 0.5,
                        Scalar(0, 128, 255), 2);
        }

        imshow("Results", work_image);
        waitKey();
    } while (false);
}


int main(int argc, char **argv) {
    // getting parameters and prepare workspace
    workspace ws;
    if (argc < 4) {
        cerr << "usage: hw4 <camera_id> <orig_x> <orig_y> [ <scaling> ]";
        return 1;
    }
    double cam_id;
    double origin_x;
    double origin_y;
    double scaling;
    try {
        cam_id = atoi(argv[1]);
        origin_x = atoi(argv[2]);
        origin_y = atoi(argv[3]);
        if (argc == 5) {
            scaling = atoi(argv[4]);
            assert(scaling >= 0);
        } else {
            scaling = -1.0;
        }

    } catch (int _) {
        cerr << "Error parsing parameters";
        return 1;
    }

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


    // Connect to camera
    cout << "Connecting to camera..." << endl;
    VideoCapture camera = init_camera(cam_id);
    waitKey(1000);

    // Get scaling factor TODO @Marian whats the point of sf_y?
    double sf_x, sf_y;
    if (argc == 5) {
        sf_x = scaling;
        sf_y = -sf_x;
    } else {
        double square_px;
        const int square_size = 5;
        bool measured = measure_ground_squares(camera, &square_px, square_size, Size(8, 6));

        if (measured) {
            cout << square_size << "mm is ca. " << square_px << " pixels in camera space." << endl;
        } else {
            cerr << "Failed to measure the ground scaling." << endl;
            return 2;
        }

        sf_y = square_size / square_px;
        sf_x = -sf_y;
    }
    prepare_workspace(origin_x, origin_y, CAM_Z, CAM_TH, CAM_PSI, CAM_PHI,
                      sf_x, sf_y,
                      camera_matrix, dist_coeffs,
                      &ws);
    cout << "Scaling factor is [" << sf_x << ", " << sf_y << "]" << endl;
    cout << "Assuming the camera origin [0, 0] is at ["
         << ws.d_x << ", " << ws.d_y << ", " << ws.d_z
         << "] in robot space." << endl;
    // -------------------- Workspace prepared performing tests
    test_p_transform(&ws);
    test_centroid_computation(camera, ws);
    return 0;
}


