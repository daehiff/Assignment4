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

#define PI 3.141592653589793238463

using namespace std;

typedef unordered_map<long long, bool> THashMap;

long long coords_to_hash(int a, int b) {
    long long a_ = a >= 0 ? 2 * a : -2 * a - 1;
    long long b_ = b >= 0 ? 2 * b : -2 * b - 1;
    return (a_ + b_) * (a_ + b_ + 1) / 2 + a_;
}

double get_central_moments(vector<tuple<int, int>> *max_area, cv::Point2f *centroid, int k, int j) {
    assert(k >= 0 && j >= 0);
    double moment = 0.0;
    for (auto &i : *max_area) {
        int x = get<1>(i);
        int y = get<0>(i);
        moment += pow(x - centroid->x, k) * pow(y - centroid->y, j);
    }
    return moment / ((double) (k + j + 2.0) / 2.0);
}

double get_pa_angle(vector<tuple<int, int>> *max_area, cv::Point2f *centroid) {
    double pa_11 = get_central_moments(max_area, centroid, 1, 1);
    double pa_20 = get_central_moments(max_area, centroid, 2, 0);
    double pa_02 = get_central_moments(max_area, centroid, 0, 2);
    // this does not conform with the lecture but gives the best results :D
    return 0.5 * atan2(2 * pa_11, pa_02 - pa_20);
}

cv::Point2f *get_centoid(vector<tuple<int, int>> *max_area) {
    double centr_x = 0.0;
    double centr_y = 0.0;
    for (auto &i : *max_area) {
        int x = get<1>(i);
        int y = get<0>(i);
        centr_x += x;
        centr_y += y;
        //colorImg.at<cv::Vec3f>(x, y) = cv::Vec3f(0.0, 120.0, 0);
    }
    centr_x = centr_x / max_area->size();
    centr_y = centr_y / max_area->size();
    return new cv::Point2f(centr_x, centr_y);
}


THashMap *fill_map(int rows, int cols) {
    auto *vc = new THashMap();
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            (*vc)[coords_to_hash(i, j)] = true;
        }
    }
    assert(vc->size() == (unsigned int) (rows * cols));
    return vc;
}


void discover_area(cv::Mat *image, tuple<int, int> seed, THashMap *vc, vector<tuple<int, int>> *out_area) {
    queue<tuple<int, int>> img_area;
    img_area.push(seed);
    while (!img_area.empty()) {
        auto tmp = img_area.front();
        int i = get<0>(tmp);
        int j = get<1>(tmp);
        img_area.pop();
        if (i < image->rows && j < image->cols
            && vc->at(coords_to_hash(i, j))
            && image->at<uchar>(i, j) == 255) {
            out_area->push_back(tmp);
            (*vc)[coords_to_hash(i, j)] = false;
            // insert all neighbors of tmp
            img_area.push(make_tuple(i + 1, j));
            img_area.push(make_tuple(i + 1, j + 1));
            img_area.push(make_tuple(i, j + 1));
            img_area.push(make_tuple(i - 1, j + 1));
            img_area.push(make_tuple(i - 1, j));
            img_area.push(make_tuple(i - 1, j - 1));
            img_area.push(make_tuple(i, j - 1));
            img_area.push(make_tuple(i + 1, j - 1));
        }

    }
}


void get_areas(cv::Mat *image, vector<vector<tuple<int, int>>> *max_area) {
    auto *vc = fill_map(image->rows, image->cols);
    vector<tuple<int, int>> area;
    for (int i = 0; i < image->rows; ++i) {
        for (int j = 0; j < image->cols; ++j) {
            int intensity = image->at<uchar>(i, j);
            const long long key = coords_to_hash(i, j);
            if (intensity != 0) {
                if (vc->at(key)) {
                    area.clear();
                    discover_area(image, make_tuple(i, j), vc, &area);
                    if (area.size() > 1000) { // everything below 100 px is considered as noise
                        max_area->push_back(area);
                    }
                }
            } else {
                if (vc->at(key)) {
                    (*vc)[key] = false;
                }

            }


        }
    }
}

/**
 * From a grayscaled image perform a segmentation and them comput the centroid and PA of each segment
 * @param workImage
 * @param centroids
 * @return
 */
int get_centroids(cv::Mat *workImage, vector<tuple<double, double, double>> *centroids) {
    cv::GaussianBlur(*workImage, *workImage, cv::Size(3, 3), 0, 0);
    cv::threshold(*workImage, *workImage, 200, 255, cv::THRESH_BINARY);
    cv::erode(*workImage, *workImage, cv::Mat());
    cv::dilate(*workImage, *workImage, cv::Mat());
    vector<vector<tuple<int, int>>> areas;

    get_areas(workImage, &areas);
    for (auto max_area: areas) {
        auto centroid = get_centoid(&max_area);
        double pa_angle_1 = get_pa_angle(&max_area, centroid);
        auto tmp = new tuple<double, double, double>(centroid->x, centroid->y, pa_angle_1);
        centroids->push_back(*tmp);
    }
    return 0;
}