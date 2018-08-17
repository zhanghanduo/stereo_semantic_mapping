//
// Created by hd on 1/10/18.
//

#ifndef PROJECT_MAP_ADD_H
#define PROJECT_MAP_ADD_H
#include <iostream>
#include <sstream>
#include <cstring>
#include <fstream>
#include <cmath>
#include <ctime>
// Thirdparty
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
//#include <std_msgs/String.h>
//#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// Custom
#include "obstacle_ind_merge.h"
#include "map_publisher.h"
#include "obstacle_merge.h"
#include <obstacle_msgs/MapInfo.h>
#include <obstacle_msgs/obs.h>
//#include <obstacle_msgs/point3.h>
#include "util.h"
#include "tracked_obstacle.h"

class obstacle_ind_merge;
class Obstacle_merge;
class MapPublisher;

class MapAdd {
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapAdd( ros::NodeHandle nh, obstacle_ind_merge *pObstacle_ind_merge,
            Obstacle_merge *pObstacle_merge, MapPublisher *pMapPublisher, int input_source);

    ~MapAdd();

    void addMapInfo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&,
                    const obstacle_msgs::MapInfo::ConstPtr&,
                    const obstacle_msgs::MapInfo::ConstPtr&);

    void addMapSingle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&, const obstacle_msgs::MapInfo::ConstPtr& );

    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);

    void map_wide_callback( const obstacle_msgs::MapInfo::ConstPtr& );

    void map_long_callback( const obstacle_msgs::MapInfo::ConstPtr& );

    double smooth_thres;
private:

    Util::CPPTimer count_timer_;
    int input_source_;   // 0: both; 1: wide_camera; 2: long_camera

//    ros::Publisher gps_pub_;

//    unsigned int counter;

    Eigen::Isometry3d pose_local;

    double pose_timestamp_, long_timestamp_, wide_timestamp_;

    // obstacle individual merge
    obstacle_ind_merge *mpObstacle_ind_merge;

    // obstacles merge
    Obstacle_merge *mpObstacle_merge;

    MapPublisher *mpMapPublisher;

//
//    double obstacleCostFunction(const MapElement& new_obstacle, const TrackedObstacle& old_obstacle);
//    double obstacleCostFunction(const MapElement& new_obstacle, const MapElement& old_obstacle);
//    void calculateCostMatrix(const std::vector<MapElement*>& new_obstacles, arma::mat& cost_matrix);
//    void calculateRowMinIndices(const arma::mat& cost_matrix, std::vector<int>& row_min_indices);
//    void calculateColMinIndices(const arma::mat& cost_matrix, std::vector<int>& col_min_indices);
//
//    bool fusionObstacleUsed(const int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
//    bool fusionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
//    bool fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
//    bool fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);
//
//    void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
//                       std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);
//    void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
//                         std::vector<TrackedObstacle>& new_tracked, const Obstacles::ConstPtr& new_obstacles);
};

#endif //PROJECT_MAP_ADD_H
