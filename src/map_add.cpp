//
// Created by hd on 1/10/18.
//
#include "map_add.h"
#include <eigen_conversions/eigen_msg.h>

using namespace std;

MapAdd::MapAdd(ros::NodeHandle nh, obstacle_ind_merge *pObstacle_ind_merge,
               Obstacle_merge *pObstacle_merge, MapPublisher *pMapPublisher, int input_source)
        :mpObstacle_ind_merge(pObstacle_ind_merge), mpObstacle_merge(pObstacle_merge),
         mpMapPublisher(pMapPublisher), input_source_(input_source){

//    mpMapPublisher->timer_.setPeriod(ros::Duration(0.1), false);
//    mpMapPublisher->timer_.start();
    nh.param<double>("smooth_value", smooth_thres, 0.8);

    mpObstacle_ind_merge->update_threshold(smooth_thres);
//    gps_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/local_pose", 100);
}

void MapAdd::addMapInfo(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_,
                       const obstacle_msgs::MapInfo::ConstPtr& wide_map_,
                       const obstacle_msgs::MapInfo::ConstPtr& long_map_){

    geometry_msgs::PoseWithCovarianceStamped pose_stamp(*pose_);

    obstacle_msgs::MapInfo wide_map_stamp(*wide_map_);

    obstacle_msgs::MapInfo long_map_stamp(*long_map_);

    pose_timestamp_ = pose_stamp.header.stamp.toSec();

    Eigen::Isometry3d pose_local;

    tf::poseMsgToEigen(pose_stamp.pose.pose, pose_local);

    mpMapPublisher->SetCurrentCameraPose(pose_local);

//    mpObstacle_merge->tempMapSet_long.clear();
//
//    mpObstacle_merge->tempMapSet.clear();

    mpObstacle_merge->frame_num ++;
    double pitch = wide_map_stamp.pitch;

    for (auto &i : long_map_stamp.obsData) {

            Eigen::Vector3d mTc_long(i.centerPos.x, i.centerPos.y, i.centerPos.z);

            Eigen::Vector3d mTc2w_long = pose_local * mTc_long;

            MapElement *long_map_element;

            long_map_element = new MapElement(i.identityID, pose_timestamp_, mTc2w_long, mTc_long, i.height, i.diameter, i.probability, i.classes, i.histogram);

//            mpObstacle_merge->tempMapSet_long.push_back(long_map_element);

            mpObstacle_ind_merge->add(sensor_type.long_camera, long_map_element, i.identityID);

    }

    for (auto &i : wide_map_stamp.obsData) {

            Eigen::Vector3d mTc_wide(i.centerPos.x, i.centerPos.y, i.centerPos.z);

            Eigen::Vector3d mTc2w_wide = pose_local * mTc_wide;

            MapElement *wide_map_element;

            wide_map_element = new MapElement(i.identityID, pose_timestamp_, mTc2w_wide, mTc_wide, i.height, i.diameter, i.probability, i.classes, i.histogram);

//            mpObstacle_merge->tempMapSet.push_back(wide_map_element);

            mpObstacle_ind_merge->add(sensor_type.wide_camera, wide_map_element, i.identityID);

    }

    mpObstacle_merge->roadSlopeSet.push_back(pitch);

//    mpObstacle_merge->tempo_obstacleCallback();
    mpMapPublisher->SetCurrentRoadAngle(pitch);

    mpMapPublisher->Refresh(pose_stamp.header);

}

void MapAdd::addMapSingle(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_,
                          const obstacle_msgs::MapInfo::ConstPtr& map_){
    obstacle_msgs::MapInfo map_stamp(*map_);

    geometry_msgs::PoseWithCovarianceStamped pose_stamp(*pose_);      //camera pose

    pose_timestamp_ = map_stamp.header.stamp.toSec();

    Eigen::Isometry3d pose_camera; //, pose_eigen

    tf::poseMsgToEigen(pose_stamp.pose.pose, pose_camera);

//    gps_to_local(pose_eigen, pose_camera);

    /***************************************************
     * This code block is used to generate new pose msg
     */

//    geometry_msgs::PoseWithCovarianceStamped msg;
//    geometry_msgs::Pose local_pose_msg;
//
//    msg.header.stamp = pose_stamp.header.stamp;
//    msg.header.frame_id = "/world";
//
//    tf::poseEigenToMsg(pose_camera, local_pose_msg);
//
//    msg.pose.pose = local_pose_msg;
//
//    gps_pub_.publish(msg);

    /****************************************************/

    mpMapPublisher->SetCurrentCameraPose(pose_camera);

    double pitch = map_stamp.pitch;

//    mpObstacle_merge->tempMapSet.clear();

    mpObstacle_merge->frame_num ++;
    for (auto &i : map_stamp.obsData) {

        Eigen::Vector3d mTc_long(i.centerPos.x, i.centerPos.y, i.centerPos.z);

        Eigen::Vector3d mTc2w_long = pose_camera * mTc_long;

        MapElement *map_element;

        map_element = new MapElement(i.identityID, pose_timestamp_, mTc2w_long, mTc_long, i.height, i.diameter, i.probability, i.classes, i.histogram);

//            mpObstacle_merge->tempMapSet_long.push_back(long_map_element);

        if(input_source_ == 1){

            mpObstacle_ind_merge->add(sensor_type.wide_camera, map_element, i.identityID);
        }
        else if(input_source_ == 2) {
            mpObstacle_ind_merge->add(sensor_type.long_camera, map_element, i.identityID);
        }
        else{
            ROS_ERROR("Need to run cmerge node instead of single node!");
            return ;
        }

    }
//    for (auto &i : map_stamp.obsData) {
//
//        Eigen::Vector3d mTc(i.centerPos.x, i.centerPos.y, i.centerPos.z);
//
//        Eigen::Vector3d mTc2w = pose_camera * mTc;
//
//        MapElement *map_element;
//
//        map_element = new MapElement(i.identityID, pose_timestamp_, mTc2w, mTc, i.height, i.diameter, i.probability, i.classes, i.histogram);
//
//        mpObstacle_merge->tempMapSet.push_back(map_element);
//
//        mpObstacle_ind_merge->add(sensor_type.wide_camera, map_element, i.identityID);
//    }
    mpObstacle_merge->roadSlopeSet.push_back(pitch);
//    mpObstacle_merge->tempo_obstacleCallback();
//
    mpMapPublisher->SetCurrentRoadAngle(pitch);
    mpMapPublisher->Refresh(pose_stamp.header);
}

MapAdd::~MapAdd() = default;

void MapAdd::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_){

    ROS_WARN("pose callback triggered!");

    geometry_msgs::PoseWithCovarianceStamped pose_stamp(*pose_);      //camera pose

    pose_timestamp_ = pose_stamp.header.stamp.toSec();

    tf::poseMsgToEigen(pose_stamp.pose.pose, pose_local);

    mpMapPublisher->SetCurrentCameraPose(pose_local);

}

void MapAdd::map_long_callback( const obstacle_msgs::MapInfo::ConstPtr& map_long_ ){

    ROS_WARN("long map callback triggered!");

    obstacle_msgs::MapInfo map_stamp(*map_long_);

    for (auto &i : map_stamp.obsData) {

        Eigen::Vector3d mTc(i.centerPos.x, i.centerPos.y, i.centerPos.z);

        Eigen::Vector3d mTc2w = pose_local * mTc;

        MapElement *map_element;

        map_element = new MapElement(i.identityID, pose_timestamp_, mTc2w, mTc, i.height, i.diameter, i.probability, i.classes, i.histogram);

        mpObstacle_merge->tempMapSet.push_back(map_element);

        mpObstacle_ind_merge->add(sensor_type.wide_camera, map_element, i.identityID);
    }


    mpObstacle_merge->tempMapSet_long.clear();

}

void MapAdd::map_wide_callback( const obstacle_msgs::MapInfo::ConstPtr& map_wide ){

    ROS_WARN("wide map callback triggered!");

    mpObstacle_merge->tempMapSet.clear();
}