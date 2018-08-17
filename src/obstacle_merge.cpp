//
// Created by Handuo on 4/22/18.
//

#include "obstacle_merge.h"
#include "obstacle_ind_merge.h"

class obstacle_ind_merge;

Obstacle_merge::Obstacle_merge():N(0),T(0),U(0),frame_num(0){

    tracked_obstacles_.clear();
    untracked_obstacles_.clear();
};

void Obstacle_merge::timerCallback(const ros::TimerEvent&) {
    predictObstaclesState();
//    publishObstacles();
}

void Obstacle_merge::tempo_obstacleCallback() {

    N = static_cast<int>(tempMapSet.size());
    T = static_cast<int>(tracked_obstacles_.size());
    U = static_cast<int>(untracked_obstacles_.size());

     if (T + U == 0) {
        untracked_obstacles_ = tempMapSet;
    }

    Eigen::MatrixXd cost_matrix;
    cost_matrix.setZero(N, T + U);
    calculateCostMatrix(tempMapSet, cost_matrix);

    std::vector<int> row_min_indices;
    calculateRowMinIndices(cost_matrix, row_min_indices);

    std::vector<int> col_min_indices;
    calculateColMinIndices(cost_matrix, col_min_indices);

    std::vector<int> used_old_obs, used_new_obs;

    std::vector<TrackedObstacle*> new_tracked_obstacles;
    std::vector<MapElement*> new_untracked_obstacles;

    // Check for fusion (only for tracked obstacles)
    for (int i =0 ; i < T-1; ++i) {
        if (fusionObstacleUsed(i, col_min_indices, used_new_obs, used_old_obs))
            continue;

        std::vector<int> fusion_indices;
        fusion_indices.push_back(i);

        for (int j = i+1; j < T; ++j) {
            if (fusionObstaclesCorrespond(i, j, col_min_indices, used_old_obs))
                fusion_indices.push_back(j);
        }

        if (fusion_indices.size() > 1) {
            fuseObstacles(fusion_indices, col_min_indices, new_tracked_obstacles);

            // Mark used old and new obstacles
            used_old_obs.insert(used_old_obs.end(), fusion_indices.begin(), fusion_indices.end());
            used_new_obs.push_back(col_min_indices[i]);
        }
    }

    // Check for fission (only tracked obstacles)
    for (int i = 0; i < N-1; ++i) {
        if (fissionObstacleUsed(i, T, row_min_indices, used_new_obs, used_old_obs))
            continue;

        std::vector<int> fission_indices;
        fission_indices.push_back(i);

        for (int j = i+1; j < N; ++j) {
            if (fissionObstaclesCorrespond(i, j, row_min_indices, used_new_obs))
                fission_indices.push_back(j);
        }

        if (fission_indices.size() > 1) {
            fissureObstacle(fission_indices, row_min_indices, new_tracked_obstacles);

            // Mark used old and new obstacles
            used_old_obs.push_back(row_min_indices[i]);
            used_new_obs.insert(used_new_obs.end(), fission_indices.begin(), fission_indices.end());
        }
    }

    // Check for other possibilities
    for (int n = 0; n < N; ++n) {
        // Discard nth obstacle if it was already used
        if (std::find(used_new_obs.begin(), used_new_obs.end(), n) != used_new_obs.end())
            continue;

        // Mark nth obstacle as untracked if no correspondence was found
        if (row_min_indices[n] == -1) {
            new_untracked_obstacles.push_back(tempMapSet[n]);
        }
            // Check if the corresponding obstacle was used
        else if (std::find(used_old_obs.begin(), used_old_obs.end(), row_min_indices[n]) == used_old_obs.end()) {
            // Correct already tracked obstacle
            if (row_min_indices[n] >= 0 && row_min_indices[n] < T) {
                tracked_obstacles_[row_min_indices[n]] -> correctState(tempMapSet[n]);
            }
                // Create a new tracked obstacle
            else if (row_min_indices[n] >= T) {
                auto to = new TrackedObstacle(untracked_obstacles_[row_min_indices[n] - T]);
//                TrackedObstacle to(untracked_obstacles_[row_min_indices[n] - T]);
                to -> correctState(tempMapSet[n]);
                for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
                    to -> updateState();
                new_tracked_obstacles.push_back(to);
            }

            used_new_obs.push_back(n);
        }
    }

    bObsUpdate = true;

}

bool Obstacle_merge::isObsUpdated() {
    return bObsUpdate;
}

void Obstacle_merge::ResetObsUpdated() {
    bObsUpdate = false;
}

void Obstacle_merge::predictObstaclesState() {
    for (int i = 0; i < tracked_obstacles_.size(); ++i) {
        if (!tracked_obstacles_[i]->hasFaded())
            tracked_obstacles_[i]->updateState();
        else
            tracked_obstacles_.erase(tracked_obstacles_.begin() + i--);
    }
}

//void Obstacle_merge::publishObstacles(){
//
//
//}

std::vector<MapElement*> Obstacle_merge::getCurrentMap(){
    /*
     * Convert the database in std::map format to std::vector format.
     */

    std::vector<MapElement*> map_vector;

    for (const auto &s : mapDatabase){
        map_vector.push_back(s.second);
    }
    return map_vector;
}

void Obstacle_merge::calculateCostMatrix(const std::vector<MapElement*>& new_obstacles, Eigen::MatrixXd& cost_matrix){
    /*
     * Cost between two obstacles represents their difference.
     * The bigger the cost, the less similar they are.
     * N rows of cost_matrix represent new obstacles.
     * T+U columns of cost matrix represent old tracked and untracked obstacles.
     */
    for (int n = 0; n < N; ++n) {
        for (int t = 0; t < T; ++t)
            cost_matrix(n, t) = obstacle_ind_dist_CostFunction(new_obstacles[n], tracked_obstacles_[t]);

        for (int u = 0; u < U; ++u)
            cost_matrix(n, u + T) = obstacle_ind_dist_CostFunction(new_obstacles[n], untracked_obstacles_[u]);
    }
}

double Obstacle_merge::obstacle_ind_dist_CostFunction(const MapElement* new_obstacle, const TrackedObstacle* old_obstacle) {
    Eigen::MatrixXd covariance = old_obstacle->getKF().S;

    Eigen::Vector3d v = old_obstacle->getObstacle()->velocity;

    double angle = atan2(v(1), v(0));

    double dt = 0.01;

    //TODO: Covariance propagation

    const Eigen::MatrixXd &new_covariance = covariance;

    Eigen::Vector3d mean = old_obstacle->getObstacle()->center_point_world + v * dt;

    Eigen::Vector3d state = new_obstacle->center_point_world;

    Eigen::MatrixXd cost = (state - mean).transpose() * new_covariance.inverse() * (state - mean);  // Use Mahalanobis distance

    //  cout << "Velocity:" << endl << v << endl;
    //  cout << "Angle: " << angle << endl;
    //  cout << "Covariance:" << endl << new_covariance << endl;
    //  cout << "Cost: " << cost(0, 0) << endl;

    return cost(0, 0);
}

double Obstacle_merge::obstacle_ind_dist_CostFunction(const MapElement* new_obstacle, const MapElement* old_obstacle) {
    Eigen::Vector3d mean = old_obstacle->center_point_world;

    Eigen::Vector3d state = new_obstacle->center_point_world;

    Eigen::MatrixXd cost = (state - mean).transpose() * (state - mean);  // Use Euclidean distance

    return cost(0,0);
}

void Obstacle_merge::calculateRowMinIndices(const Eigen::MatrixXd& cost_matrix, std::vector<int>& row_min_indices) {
    /*
     * Vector of row minimal indices keeps the indices of old obstacles (tracked and untracked)
     * that have the minimum cost related to each of new obstacles, i.e. row_min_indices[n]
     * keeps the index of old obstacle that has the minimum cost with n-th new obstacle.
     */
    auto N_cost = static_cast<int>(cost_matrix.rows());

    row_min_indices.assign(N_cost, -1); // Minimum index -1 means no correspondence has been found

    for (int n = 0; n < N_cost; ++n) {
        double min_cost = p_min_correspondence_cost_;

        for (int t = 0; t < T; ++t) {
            if (cost_matrix(n, t) < min_cost) {
                min_cost = cost_matrix(n, t);
                row_min_indices[n] = t;
            }
        }

        for (int u = 0; u < U; ++u) {
            if (cost_matrix(n, u + T) < min_cost) {
                min_cost = cost_matrix(n, u + T);
                row_min_indices[n] = u + T;
            }
        }
    }
}

void Obstacle_merge::calculateColMinIndices(const Eigen::MatrixXd& cost_matrix, std::vector<int>& col_min_indices) {
    /*
     * Vector of column minimal indices keeps the indices of new obstacles that has the minimum
     * cost related to each of old (tracked and untracked) obstacles, i.e. col_min_indices[i]
     * keeps the index of new obstacle that has the minimum cost with i-th old obstacle.
     */
    auto N_cost = static_cast<int>(cost_matrix.rows());

    col_min_indices.assign(T + U, -1); // Minimum index -1 means no correspondence has been found

    for (int t = 0; t < T; ++t) {
        double min_cost = p_min_correspondence_cost_;

        for (int n = 0; n < N_cost; ++n) {
            if (cost_matrix(n, t) < min_cost) {
                min_cost = cost_matrix(n, t);
                col_min_indices[t] = n;
            }
        }
    }

    for (int u = 0; u < U; ++u) {
        double min_cost = p_min_correspondence_cost_;

        for (int n = 0; n < N_cost; ++n) {
            if (cost_matrix(n, u + T) < min_cost) {
                min_cost = cost_matrix(n, u + T);
                col_min_indices[u + T] = n;
            }
        }
    }
}

bool Obstacle_merge::fusionObstacleUsed(const int idx, const std::vector<int> &col_min_indices,
                                        const std::vector<int> &used_new, const std::vector<int> &used_old) {
    /*
     * This function returns true if:
     * - idx-th old obstacle was already used
     * - obstacle to which idx-th old obstacle corresponds was already used
     * - there is no corresponding obstacle
     */

    return (std::find(used_old.begin(), used_old.end(), idx) != used_old.end() ||
            std::find(used_new.begin(), used_new.end(), col_min_indices[idx]) != used_new.end() ||
            col_min_indices[idx] < 0);
}

bool Obstacle_merge::fusionObstaclesCorrespond(const int idx, const int jdx,
                                               const std::vector<int>& col_min_indices,
                                               const std::vector<int>& used_old) {
    /*
     * This function returns true if:
     * - both old obstacles correspond to the same new obstacle
     * - jdx-th old obstacle was not yet used
     */

    return (col_min_indices[idx] == col_min_indices[jdx] &&
            find(used_old.begin(), used_old.end(), jdx) == used_old.end());
}

bool Obstacle_merge::fissionObstacleUsed(const int idx, const int T, const std::vector<int>& row_min_indices,
                                         const std::vector<int>& used_new, const std::vector<int>& used_old) {
    /*
     * This function returns true if:
     * - idx-th new obstacle was already used
     * - obstacle to which idx-th new obstacle corresponds was already used
     * - there is no corresponding obstacle
     * - obstacle to which idx-th new obstacle corresponds is untracked
     */

    return (std::find(used_new.begin(), used_new.end(), idx) != used_new.end() ||
            std::find(used_old.begin(), used_old.end(), row_min_indices[idx]) != used_old.end() ||
            row_min_indices[idx] < 0 ||
            row_min_indices[idx] >= T);
}

bool Obstacle_merge::fissionObstaclesCorrespond(const int idx, const int jdx, const std::vector<int>& row_min_indices, const std::vector<int>& used_new) {
    /*
     * This function returns true if:
     * - both new obstacles correspond to the same old obstacle
     * - jdx-th new obstacle was not yet used
     */

    return (row_min_indices[idx] == row_min_indices[jdx] &&
            find(used_new.begin(), used_new.end(), jdx) == used_new.end());
}

void Obstacle_merge::fuseObstacles(const std::vector<int>& fusion_indices, const std::vector<int> &col_min_indices,
                                   std::vector<TrackedObstacle*>& new_tracked) {
    MapElement *c = nullptr;

    double sum_var_x  = 0.0;
    double sum_var_y  = 0.0;
    double sum_var_z  = 0.0;
    double sum_var_vx = 0.0;
    double sum_var_vy = 0.0;
    double sum_var_vz = 0.0;

    //TODO: obstacle size (height, width) also should be considered!

    for (int idx : fusion_indices) {
        c->center_point_world.x() += tracked_obstacles_[idx]->getObstacle()->center_point_world.x() / tracked_obstacles_[idx]->getKF().P(0, 0);
        c->velocity.x() += tracked_obstacles_[idx]->getObstacle()->velocity.x() / tracked_obstacles_[idx]->getKF().P(1, 1);

        c->center_point_world.y() += tracked_obstacles_[idx]->getObstacle()->center_point_world.y() / tracked_obstacles_[idx]->getKF().P(2, 2);
        c->velocity.y() += tracked_obstacles_[idx]->getObstacle()->velocity.y() / tracked_obstacles_[idx]->getKF().P(3, 3);

        c->center_point_world.z() += tracked_obstacles_[idx]->getObstacle()->center_point_world.z() / tracked_obstacles_[idx]->getKF().P(4, 4);
        c->velocity.z() += tracked_obstacles_[idx]->getObstacle()->velocity.z() / tracked_obstacles_[idx]->getKF().P(5, 5);

        sum_var_x += 1.0 / tracked_obstacles_[idx]->getKF().P(0, 0);
        sum_var_vx += 1.0 / tracked_obstacles_[idx]->getKF().P(1, 1);

        sum_var_y += 1.0 / tracked_obstacles_[idx]->getKF().P(2, 2);
        sum_var_vy += 1.0 / tracked_obstacles_[idx]->getKF().P(3, 3);

        sum_var_z += 1.0 / tracked_obstacles_[idx]->getKF().P(4, 4);
        sum_var_vz += 1.0 / tracked_obstacles_[idx]->getKF().P(5, 5);
    }

    c->center_point_world.x() /= sum_var_x;
    c->center_point_world.y() /= sum_var_y;
    c->center_point_world.z() /= sum_var_z;
    c->velocity.x() /= sum_var_vx;
    c->velocity.y() /= sum_var_vy;
    c->velocity.z() /= sum_var_vz;

//    boost::shared_ptr<TrackedObstacle> to = boost::make_shared<TrackedObstacle>(c);
//    TrackedObstacle to(c);
    auto to = new TrackedObstacle(c);
    to->correctState(tempMapSet[col_min_indices[fusion_indices.front()]]);
    for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
        to->updateState();

    new_tracked.push_back(to);
}

void Obstacle_merge::fissureObstacle(const std::vector<int>& fission_indices, const std::vector<int>& row_min_indices,
                                     std::vector<TrackedObstacle*>& new_tracked) {
    // For each new obstacle taking part in fission create a tracked obstacle from the original old one and update it with the new one
    for (int idx : fission_indices) {

        TrackedObstacle* to = tracked_obstacles_[row_min_indices[idx]];
        to->correctState(tempMapSet[idx]);
        for (int i = 0; i < static_cast<int>(p_loop_rate_ / p_sensor_rate_); ++i)
            to->updateState();

        new_tracked.push_back(to);
    }
}