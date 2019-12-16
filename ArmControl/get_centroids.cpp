// This program is a simple template of an C++ program loading and showing image with OpenCV.
// You can ignore this file and write your own program.
// The program takes a image file name as an argument.

#include <stdio.h>
#include <unordered_map>
#include <tuple>
#include <vector>
#include <queue>
#include <assert.h>
#include <stdlib.h>


#define PI 3.141592653589793238463
using namespace std;

typedef unordered_map<long long, bool> THashMap;

long long coords_to_hash(int a, int b) {
    long A = a >= 0 ? 2 * a : -2 * a - 1;
    long B = b >= 0 ? 2 * b : -2 * b - 1;
    return (A + B) * (A + B + 1) / 2 + A;

}

double get_central_moments(vector<tuple<int, int>> max_area, cv::Point2f *centroid, int k, int j) {
    assert(k >= 0 && j >= 0);
    double moment = 0.0;
    for (size_t i = 0; i < max_area.size(); i++) {
        int x = get<1>((max_area)[i]);
        int y = get<0>((max_area)[i]);
        moment += pow(x - centroid->x, k) * pow(y - centroid->y, j);
    }
    return moment / ((double) (k + j + 2.0) / 2.0);
}

double get_pa_angle(vector<tuple<int, int>> max_area, cv::Point2f *centroid) {
    double pa_11 = get_central_moments(max_area, centroid, 1, 1);
    double pa_20 = get_central_moments(max_area, centroid, 2, 0);
    double pa_02 = get_central_moments(max_area, centroid, 0, 2);
    // this does not conform with the lecture but gives the best results :D
    return 0.5f * atan2(2 * pa_11, pa_02 - pa_20);
}

cv::Point2f *get_centoid(vector<tuple<int, int>> max_area) {
    long double centr_x = 0.0;
    long double centr_y = 0.0;
    for (size_t i = 0; i < max_area.size(); i++) {
        int x = get<1>((max_area)[i]);
        int y = get<0>((max_area)[i]);
        centr_x += x;
        centr_y += y;
        //colorImg.at<cv::Vec3f>(x, y) = cv::Vec3f(0.0, 120.0, 0);
    }
    centr_x = centr_x / (double) max_area.size();
    centr_y = centr_y / (double) max_area.size();
    return new cv::Point2f((float) centr_x, (float) centr_y);
}


THashMap *fill_map(int rows, int cols) {
    THashMap *vc = new THashMap();
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            (*vc)[coords_to_hash(i, j)] = true;
        }
    }
    assert(vc->size() == (unsigned int) (rows * cols));
    return vc;
}


vector<tuple<int, int>> *discover_area(cv::Mat image, tuple<int, int> seed, THashMap *vc) {
    queue<tuple<int, int>> img_area;
    vector<tuple<int, int>> *out_area = new vector<tuple<int, int>>; // init on heap
    img_area.push(seed);
    while (!img_area.empty()) {
        auto tmp = img_area.front();
        int i = get<0>(tmp);
        int j = get<1>(tmp);
        img_area.pop();
        if (vc->find(coords_to_hash(i, j)) == vc->end()) {
            continue;
        }
        if (i < image.rows && j < image.cols
            && vc->at(coords_to_hash(i, j))
            && image.at<uchar>(i, j) == 255) {
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
    return out_area;
}


vector<vector<tuple<int, int>>> *get_areas(cv::Mat image) {
    vector<vector<tuple<int, int>>> *max_area = new vector<vector<tuple<int, int>>>;
    THashMap *vc = fill_map(image.rows, image.cols);
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            int intensity = image.at<uchar>(i, j);
            auto key = coords_to_hash(i, j);
            if (vc->find(key) == vc->end()) {
                continue;
            }
            if (intensity != 0) {
                if (vc->at(key)) {
                    vector<tuple<int, int>> *area = discover_area(image, make_tuple(i, j), vc);
                    if (area->size() > 400) { // everything below 400 px is considered as noise
                        max_area->push_back(*area);
                    }
                }
            } else {
                if (vc->at(key)) {
                    (*vc)[key] = false;
                }

            }


        }
    }
    // super ugly way of selection sorting the areas if anyone wants to O(n*log(n)) it go for it :D
    for (int k = 0; k < max_area->size(); ++k) {
        vector<tuple<int, int>> tmp = (*max_area)[k];
        int min_len = (*max_area)[k].size();
        int min_idx = k;
        for (int i = k; i < max_area->size(); ++i) {
            if (min_len <= (*max_area)[i].size()) {
                min_len = (*max_area)[i].size();
                min_idx = i;
            }
        }
        (*max_area)[k] = (*max_area)[min_idx];
        (*max_area)[min_idx] = tmp;
    }
    cout << "Computed " << max_area->size() << " Centroids with sizes ordered: ";
    for (int l = 0; l < max_area->size(); ++l) {
        cout << (*max_area)[l].size() << ", ";
    }
    cout << endl;
    return max_area;
}

int get_centroids(cv::Mat workImage, vector<tuple<double, double, double>> *centroids) {
    cv::GaussianBlur(workImage, workImage, cv::Size(3, 3), 0, 0);
    cv::threshold(workImage, workImage, 164, 255, cv::THRESH_BINARY);
    cv::erode(workImage, workImage, cv::Mat());
    cv::dilate(workImage, workImage, cv::Mat());

    vector<vector<tuple<int, int>>> *areas = get_areas(workImage);
    for (size_t i = 0; i < areas->size(); i++) {
        cv::Point2f *centroid = get_centoid(((*areas)[i]));
        double pa_angle_1 = get_pa_angle(((*areas)[i]), centroid);
        auto tmp = new tuple<double, double, double>(centroid->x, centroid->y, pa_angle_1 * (180.0 / PI));
        centroids->push_back(*tmp);
    }
    return 0;
}

