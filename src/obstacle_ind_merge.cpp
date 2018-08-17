//
// Created by hd on 1/11/18.
//
#include "obstacle_ind_merge.h"
#include <deque>
#include <algorithm>

obstacle_ind_merge::obstacle_ind_merge(Obstacle_merge *pObstacle_merge):mpObstacle_merge(pObstacle_merge)
{
    mbMapUpdated = false;
    mbObsUpdated = false;
    smooth = 0.8;
}

void obstacle_ind_merge::update_threshold(double smooth_threshold){

    smooth = smooth_threshold;
}

void obstacle_ind_merge::add(unsigned char sensor, MapElement* pMap, index_t ID){
    /*
     * Add obstacle map msg to the database of obstacle mapping.
     * The same ID should represent the same obstacle by temporal sense.
     */

    MapElement *tmp_map = nullptr;
    int tmp_age_ = 0;

    if (sensor_type.long_camera == sensor) {

        if(mpObstacle_merge->mapDatabase_long.count(ID)){       //If the obstacle was already recorded in the previous frames
            tmp_age_ = mpObstacle_merge->mapDatabase_long[ID]->age;
            tmp_map = mpObstacle_merge->mapDatabase_long[ID];
        }

        mpObstacle_merge->mapDatabase_long[ID] = pMap;          //overwrite
        mpObstacle_merge->mapDatabase_long[ID]->isTracked = true;

        if((tmp_age_ > 0)&&(tmp_map)){
            mpObstacle_merge->mapDatabase_long[ID]->update_map_element(tmp_map);

            mpObstacle_merge->mapDatabase_long[ID]->set_vel(tmp_map->center_point_world,
                                                            tmp_map->cur_timestamp);

            if(tmp_age_ > 3){
                Eigen::Vector2d cur_diff, prev_diff, prev_diff_norm;

//                ROS_WARN("id: %lu | age: %d\n", ID, tmp_age_+1);
//                cur_diff = mpObstacle_merge->mapDatabase_long[ID]->younger_traj_vec;
//                cur_diff = cur_diff / cur_diff.norm();

                for(auto i=mpObstacle_merge->mapDatabase_long[ID]->traj_diff.begin();
                i!=mpObstacle_merge->mapDatabase_long[ID]->traj_diff.end() - 1; i++)
                {
                    prev_diff += *i;
                }
                prev_diff /= mpObstacle_merge->mapDatabase_long[ID]->traj_diff.size() - 1;

//                prev_diff = mpObstacle_merge->mapDatabase_long[ID]->traj_diff.rend()[1];
//                prev_diff = mpObstacle_merge->mapDatabase_long[ID]->older_traj_vec;
                prev_diff_norm = prev_diff / prev_diff.norm();

                cur_diff = mpObstacle_merge->mapDatabase_long[ID]->traj_diff.back();
                cur_diff /= cur_diff.norm();

                double cosine_diff = cur_diff.dot(prev_diff_norm);

//                if(mpObstacle_merge->mapDatabase_long[ID]->category=="truck") {
//                    printf("id: %lu | age: %d\n", ID, tmp_age_+1);
//                    printf("curr diff: %lf, %lf\n", cur_diff.x(), cur_diff.y());
//                    printf("prev diff: %lf, %lf\n", prev_diff.x(), prev_diff.y());
//                    printf("cosine diff: %lf\n", cosine_diff);
//                }
                if((cosine_diff < smooth) && (!mpObstacle_merge->mapDatabase_long[ID]->same_point) ) {   // outlier
                    mpObstacle_merge->mapDatabase_long[ID]->use_prediction(tmp_map->center_point_world);
//                    mpObstacle_merge->mapDatabase_long[ID]->use_prediction_increment(tmp_map->center_point_world, prev_diff);
//                    ROS_WARN("id: %lu | age: %d\n", ID, tmp_age_+1);
//                    ROS_WARN("curr diff: %lf, %lf\n", cur_diff.x(), cur_diff.y());
//                    ROS_WARN("prev diff: %lf, %lf\n", prev_diff.x(), prev_diff.y());
//                    ROS_WARN("cosine diff: %lf\n", cosine_diff);
                }
            }
        }
        mpObstacle_merge->mapDatabase_long[ID]->age = tmp_age_ + 1;

//        auto hist_size = static_cast<int>(pMap->histograms.size());
//
//        cv::Mat hog_mat = cv::Mat(cv::Size(hist_size, 1), CV_32FC1 );
//
//        memcpy(hog_mat.data, pMap->histograms.data(), hist_size* sizeof(float));

        mbObsUpdated = true;

//        std::cout << "long map : " << ID << " | " << mapDatabase_long[ID]->center_point_world.y() << std::endl;

    } else if(sensor_type.wide_camera == sensor){

        if(mpObstacle_merge->mapDatabase.count(ID)){  //If the obstacle was already recorded in the previous frames
            tmp_age_ = mpObstacle_merge->mapDatabase[ID]->age;
//            tmp_old_pos_ = mpObstacle_merge->mapDatabase[ID]->center_point_world;
//            tmp_old_time = mpObstacle_merge->mapDatabase[ID]->cur_timestamp;
        }

//        mpObstacle_merge->mapDatabase[ID] = pMap;   // Overwrite the previous map if already exists
        mpObstacle_merge->mapDatabase.insert(std::make_pair(ID, pMap));
//        if(mpObstacle_merge->mapDatabase[ID]->age > 1) {
         if(tmp_age_ > 0){
             mpObstacle_merge->mapDatabase[ID]->age = tmp_age_ + 1;
//             mpObstacle_merge->mapDatabase[ID]->set_vel(tmp_old_pos_, tmp_old_time);
//            std::cout << "wide age: " << mpObstacle_merge->mapDatabase[ID]->age
//                      << " | diff: " << mpObstacle_merge->mapDatabase[ID]->temp_diff << std::endl;
        }else{
             mpObstacle_merge->mapDatabase[ID]->age ++;
         }

//        mpObstacle_merge->mapDatabase[ID]->old_center_world = mpObstacle_merge->mapDatabase[ID]->center_point_world;
//        mpObstacle_merge->mapDatabase[ID]->last_timestamp = mpObstacle_merge->mapDatabase[ID]->cur_timestamp;

//        auto hist_size = static_cast<int>(pMap->histograms.size());
//
//        cv::Mat hog_mat = cv::Mat(cv::Size(hist_size, 1), CV_32FC1 );
//
//        memcpy(hog_mat.data, pMap->histograms.data(), hist_size* sizeof(float));

        mbObsUpdated = true;

//        std::cout << "wide map : " << ID << " | "
//                  << mpObstacle_merge->mapDatabase[ID]->center_point_world.y() << std::endl;
    }
}

void obstacle_ind_merge::spatialMerge() {


}

bool obstacle_ind_merge::isObsUpdated() {
    return mbObsUpdated;
}

void obstacle_ind_merge::ResetObsUpdated() {
    mbObsUpdated = false;
}

void obstacle_ind_merge::vecToMat(const std::vector<float> & hog_vec, cv::Mat & hog_mat) {

    //ignore assert of size alignment
    memcpy(hog_mat.data, hog_vec.data(), hog_vec.size()* sizeof(float));
}



template<typename T>
T obstacle_ind_merge::computeHOGHistogramDistances(
        const cv::Mat &patch, std::vector<cv::Mat> &imageHOG,
        const int distance_type) {
    T sum = 0.0;
    T argMinDistance = FLT_MAX;
    for (const auto &img_hog : imageHOG) {
        T d = cv::compareHist(patch, img_hog, distance_type);
        if (d < argMinDistance) {
            argMinDistance = static_cast<double>(d);
        }
    }
    sum = static_cast<T>(argMinDistance);
    return static_cast<T>(sum);
}







