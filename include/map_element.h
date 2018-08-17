//
// Created by hd on 2/2/18.
//

#ifndef PROJECT_MAP_ELEMENT_H
#define PROJECT_MAP_ELEMENT_H

#include <vector>
#include <Eigen/Dense>
#include "util.h"
#include <deque>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef unsigned long int index_t;

class MapElement
{
public:
    MapElement();

    MapElement(index_t, double, Eigen::Vector3d&, Eigen::Vector3d&, float, float, float, std::string&, std::vector<float>&);

    void use_prediction(Eigen::Vector3d& prev_pos);

    void use_prediction_increment(Eigen::Vector3d& prev_pos, Eigen::Vector2d& incre);

    void update_map_element(MapElement*);

//    void update_category(std::string&);

    void set_merge();

    void set_age(int );

    void set_vel(Eigen::Vector3d&, double);

    index_t id;

    Eigen::Vector3d center_point_world;

    Eigen::Vector3d center_point_local;

    Eigen::Vector3d velocity;

    std::deque<Eigen::Vector2d> traj_diff;

//    Eigen::Vector2d older_traj_vec;
//    Eigen::Vector2d younger_traj_vec;

    Eigen::Quaterniond world_orient;

    double temp_diff, cur_timestamp;

    int age;

    float height;

    float width;

    float probability;

    std::string category;

    std::vector< float > histograms;

    bool isMerged;

    bool isTracked;
    bool same_point;
};

#endif //PROJECT_MAP_ELEMENT_H
