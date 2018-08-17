//
// Created by Handuo on 4/22/18.
//

#ifndef PROJECT_OBSTACLE_MERGE_H
#define PROJECT_OBSTACLE_MERGE_H

#include <vector>
#include "tracked_obstacle.h"
#include "map_element.h"
#include "util.h"


class Obstacle_merge
{
public:

    Obstacle_merge();

    void timerCallback(const ros::TimerEvent&);

    void tempo_obstacleCallback();

    bool isObsUpdated();

    void ResetObsUpdated();

    unsigned int frame_num;

    std::vector<MapElement*> getCurrentMap();

    std::vector<MapElement *> tempMapSet_long;

    std::vector<MapElement *> tempMapSet;

    std::vector<double> roadSlopeSet;

    std::map<index_t, MapElement *> mapDatabase;

    std::map<index_t, MapElement *> mapDatabase_long;

    std::map<index_t, MapElement *> mapDatabase_merge;

    std::vector<TrackedObstacle*> tracked_obstacles_;

    std::vector<MapElement*> untracked_obstacles_;

private:

    double obstacle_ind_dist_CostFunction(const MapElement* new_obstacle, const TrackedObstacle* old_obstacle);

    double obstacle_ind_dist_CostFunction(const MapElement* new_obstacle, const MapElement* old_obstacle);

    void calculateCostMatrix(const std::vector<MapElement*>& new_obstacles, Eigen::MatrixXd& cost_matrix);

    void calculateRowMinIndices(const Eigen::MatrixXd& cost_matrix, std::vector<int>& row_min_indices);

    void calculateColMinIndices(const Eigen::MatrixXd& cost_matrix, std::vector<int>& col_min_indices);

    bool fusionObstacleUsed(int idx, const std::vector<int>& col_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
    bool fusionObstaclesCorrespond(int idx, int jdx, const std::vector<int>& col_min_indices, const std::vector<int>& used_old);
    bool fissionObstacleUsed(int idx, int T, const std::vector<int>& row_min_indices, const std::vector<int>& used_new, const std::vector<int>& used_old);
    bool fissionObstaclesCorrespond(int idx, int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new);

    void fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int>& col_min_indices,
                       std::vector<TrackedObstacle*>& new_tracked);

    void fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                         std::vector<TrackedObstacle*>& new_tracked);

    void predictObstaclesState();

    void publishObstacles();


    bool bObsUpdate;

    int N, T, U;

    ros::Timer timer_;

    double p_loop_rate_                 = 10.0;
    double p_sampling_time_             = 0.1;
    double p_sensor_rate_               = 10.0;

    double p_fade_in_duration_          = 0.5;
    double p_fade_out_duration_         = 2.0;

    double p_min_correspondence_cost_   = 0.3;
    double p_process_variance_          = 0.01;
    double p_process_rate_variance_     = 0.1;
    double p_measurement_variance_      = 1.0;
};




#endif //PROJECT_OBSTACLE_MERGE_H
