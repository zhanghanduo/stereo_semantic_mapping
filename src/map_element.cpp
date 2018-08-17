//
// Created by hd on 2/2/18.
//

#include "map_element.h"
using namespace Eigen;

MapElement::MapElement():id(0), age(0), height(0), width(0), probability(0), temp_diff(0), same_point(false),
                         cur_timestamp(0), category(nullptr), isMerged(false), isTracked(false)
{
    center_point_world.setZero();
    center_point_local.setZero();

    world_orient.x() = 0;
    world_orient.y() = 0;
    world_orient.z() = 0;
    world_orient.w() = 1.0;

    velocity.setZero();
//    younger_traj_vec.setOnes();
//    older_traj_vec.setOnes();
}

MapElement::MapElement(index_t ID_, double timestamp, Vector3d& w_point_, Vector3d& lo_point_, float height_, float width_,
                        float prob_, std::string& catogory_, std::vector<float>& histogram_): id(ID_),
                        age(0), height(height_), width(width_), probability(prob_), category(catogory_),
                        temp_diff(0), isMerged(false), cur_timestamp(timestamp), isTracked(false), same_point(false)
{
    center_point_world = w_point_;
    center_point_local = lo_point_;
    histograms = histogram_;
}


void MapElement::set_merge() {

    isMerged = true;
}

void MapElement::set_age(int age_) {

    age = age_;
}

void MapElement::use_prediction(Eigen::Vector3d& prev_pos) {

//    center_point_world = prev_pos + traj_diff.front();
    center_point_world = prev_pos; // + older_traj_vec;
}

void MapElement::use_prediction_increment(Eigen::Vector3d& prev_pos, Eigen::Vector2d& incre) {

//    center_point_world = prev_pos + traj_diff.front();
    center_point_world.x() = prev_pos.x() + incre.x()*0.9;
    center_point_world.z() = prev_pos.z() + incre.y()*0.9;
    center_point_world.y() = prev_pos.y();
}

void MapElement::set_vel(Eigen::Vector3d& old_pos, double old_time) {

    double d_t;

    d_t = cur_timestamp - old_time;

    Eigen::Vector2d traj_vec;
    traj_vec.x() = center_point_world.x() - old_pos.x();
    traj_vec.y() = center_point_world.z() - old_pos.z();
    traj_diff.push_back(traj_vec);
    if(traj_diff.size() > 15)
        traj_diff.pop_front();

//    older_traj_vec = younger_traj_vec;
//    younger_traj_vec.x() = center_point_world.x() - old_pos.x();
//    younger_traj_vec.y() = center_point_world.z() - old_pos.z();

    same_point = (traj_vec.x() == 0) && (traj_vec.y() == 0);

    velocity = (center_point_world - old_pos) / d_t;

//    temp_diff = (old_pos - center_point_world).squaredNorm();
    temp_diff = velocity.norm();
}

void MapElement::update_map_element(MapElement * tmp_map) {

//    older_traj_vec = tmp_map->older_traj_vec;
//    younger_traj_vec = tmp_map->younger_traj_vec;

    traj_diff = tmp_map->traj_diff;

    velocity = tmp_map->velocity;

    temp_diff = tmp_map->temp_diff;
}


