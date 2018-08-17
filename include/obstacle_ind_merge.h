//
// Created by hd on 1/11/18.
//

#ifndef PROJECT_MAP_MERGE_H
#define PROJECT_MAP_MERGE_H

#include <vector>
#include <tuple>
#include <Eigen/Dense>
#include <ceres/ceres.h>
//#include <glog/logging.h>

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

#include "util.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "map_element.h"
#include "tracked_obstacle.h"
#include "obstacle_merge.h"

class obstacle_ind_merge
{
public:
    explicit obstacle_ind_merge(Obstacle_merge *);

    bool isObsUpdated();

    void clear();

    void ResetObsUpdated();

    void spatialMerge();

    void add(unsigned char, MapElement*, unsigned long int);

    void erase(unsigned char, MapElement*);

    void update_threshold(double);

//    std::map<index_t, MapElement *> mapDatabase;
//
//    std::map<index_t, MapElement *> mapDatabase_long;
//
//    std::map<index_t, MapElement *> mapDatabase_merge;

    std::map<index_t, cv::Mat > mspDet_hog_long;

    std::map<index_t, cv::Mat > mspDet_hog_wide;

protected:

    bool mbObsUpdated;

    bool mbMapUpdated;

    double smooth;

    // obstacles merge
    Obstacle_merge *mpObstacle_merge;

    void vecToMat(const std::vector<float> &, cv::Mat &);

    template<typename T>
    T computeHOGHistogramDistances(
            const cv::Mat &, std::vector<cv::Mat> &,
            int = CV_COMP_BHATTACHARYYA);



private:





};












#endif //PROJECT_MAP_MERGE_H
