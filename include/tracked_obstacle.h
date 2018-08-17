//
// Created by hd on 4/19/18.
//

#ifndef PROJECT_TRACKED_OBSTACLE_H
#define PROJECT_TRACKED_OBSTACLE_H

#include <Eigen/Dense>
#include <utility>
#include "util.h"
#include "kalman.h"
#include "map_element.h"

class TrackedObstacle {
public:
    explicit TrackedObstacle(MapElement* obstacle):
            obstacle_(obstacle), kf_(0, 3, 6)
    {
        fade_out_counter_ = s_fade_counter_size_;
        initKF();
    }

    void predictState() {

        kf_.predictState();

        obstacle_->center_point_world.x() = kf_.q_pred(0);
        obstacle_->velocity.x() = kf_.q_pred(1);

        obstacle_->center_point_world.y() = kf_.q_pred(2);
        obstacle_->velocity.y() = kf_.q_pred(3);

        obstacle_->center_point_world.z() = kf_.q_pred(4);
        obstacle_->velocity.z() = kf_.q_pred(5);

//        obstacle_.height = kf_.q_pred(6);
//        obstacle_.width = kf_.q_pred(7);
        fade_out_counter_--;
    }

    void correctState(const MapElement* new_obstacle) {
        kf_.measure_sig(0) = new_obstacle->center_point_world.x();
        kf_.measure_sig(1) = new_obstacle->center_point_world.y();
        kf_.measure_sig(2) = new_obstacle->center_point_world.z();
//        kf_.measure_sig(3) = new_obstacle.height;
//        kf_.measure_sig(4) = new_obstacle.width;

        kf_.correctState();

        obstacle_->center_point_world.x() = kf_.q_est(0);
        obstacle_->velocity.x() = kf_.q_est(1);

        obstacle_->center_point_world.y() = kf_.q_est(2);
        obstacle_->velocity.y() = kf_.q_est(3);

        obstacle_->center_point_world.z() = kf_.q_est(4);
        obstacle_->velocity.z() = kf_.q_est(5);

//        obstacle_.height = kf_.q_est(6);
//        obstacle_.width = kf_.q_est(7);

        fade_out_counter_ = s_fade_counter_size_;
    }

    void updateState() {
        kf_.predictState();
        kf_.correctState(); // Measurements are frozen

        obstacle_->center_point_world.x() = kf_.q_est(0);
        obstacle_->velocity.x() = kf_.q_est(1);

        obstacle_->center_point_world.y() = kf_.q_est(2);
        obstacle_->velocity.y() = kf_.q_est(3);

        obstacle_->center_point_world.z() = kf_.q_est(4);
        obstacle_->velocity.z() = kf_.q_est(5);

//        obstacle_.height = kf_.q_est(6);
//        obstacle_.width = kf_.q_est(7);

        fade_out_counter_--;
    }

    void setSamplingTime(double tp) {
        s_sampling_time_ = tp;
    }

    void setCounterSize(int size) {
        s_fade_counter_size_ = size;
    }

    void setCovariances(double process_var, double process_rate_var, double measurement_var) {
        s_process_variance_ = process_var;
        s_process_rate_variance_ = process_rate_var;
        s_measurement_variance_ = measurement_var;
    }

    bool hasFaded() const { return fade_out_counter_ <= 0; }
    const MapElement* getObstacle() const { return obstacle_; }
    KalmanFilter getKF() const { return kf_; }

private:

    int fade_in_counter_{};

    int fade_out_counter_;

    int s_fade_counter_size_        = 200;
    double s_sampling_time_         = 0.01;
    double s_process_variance_      = 0.01;
    double s_process_rate_variance_ = 0.1;
    double s_measurement_variance_  = 1.0;

    MapElement* obstacle_;

    KalmanFilter kf_;

    void initKF() {
        kf_.state_m(0, 1) = s_sampling_time_;   //delta_t
        kf_.state_m(2, 3) = s_sampling_time_;
        kf_.state_m(4, 5) = s_sampling_time_;

        kf_.output_m(0, 0) = 1.0;
        kf_.output_m(1, 2) = 1.0;
        kf_.output_m(2, 4) = 1.0;

        kf_.R *= s_measurement_variance_;

        kf_.Q(0, 0) = s_process_variance_;
        kf_.Q(1, 1) = s_process_rate_variance_;
        kf_.Q(2, 2) = s_process_variance_;
        kf_.Q(3, 3) = s_process_rate_variance_;
        kf_.Q(4, 4) = s_process_variance_;
        kf_.Q(5, 5) = s_process_rate_variance_;
//        kf_.Q(6, 6) = s_process_variance_;
//        kf_.Q(7, 7) = s_process_variance_;

        kf_.q_est(0) = kf_.q_pred(0) = obstacle_->center_point_world.x();
        kf_.q_est(1) = kf_.q_pred(1) = obstacle_->velocity.x();
        kf_.q_est(2) = kf_.q_pred(2) = obstacle_->center_point_world.y();
        kf_.q_est(3) = kf_.q_pred(3) = obstacle_->velocity.y();
        kf_.q_est(4) = kf_.q_pred(4) = obstacle_->center_point_world.z();
        kf_.q_est(5) = kf_.q_pred(5) = obstacle_->velocity.z();
//        kf_.q_est(6) = kf_.q_pred(6) = obstacle_.height;
//        kf_.q_est(7) = kf_.q_pred(7) = obstacle_.width;
    }



};


#endif //PROJECT_TRACKED_OBSTACLE_H
