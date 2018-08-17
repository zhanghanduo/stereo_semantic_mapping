//
// Created by hd on 1/11/18.
//

#ifndef PROJECT_MAP_PUBLISHER_H
#define PROJECT_MAP_PUBLISHER_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "obstacle_ind_merge.h"
#include "obstacle_merge.h"
#include "util.h"
#include <Eigen/Dense>

//namespace rvt = rviz_visual_tools;

class MapPublisher{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MapPublisher(ros::NodeHandle, ros::NodeHandle, obstacle_ind_merge *, Obstacle_merge*);

    void Refresh(std_msgs::Header &header);

    void timerCallback(const ros::TimerEvent&);

    void PublishCurrentCamera(const Eigen::Isometry3d &Tcw, std_msgs::Header &header);

    void SetCurrentCameraPose(const Eigen::Isometry3d &Tc2w);

    void SetCurrentRoadAngle(double pitch);

    Eigen::Isometry3d GetCurrentCameraPose();

    ros::Timer timer_;

private:

    ros::NodeHandle n_local_;

    void PublishObs_long(const std::map<unsigned long int, MapElement* > &mObs, std_msgs::Header &header);

    void PublishObs_wide(const std::map<unsigned long int, MapElement* > &mObs, std_msgs::Header &header);

    void Publish_road_arrow(double, std_msgs::Header &header);

    void publish_trackedObs(const std::vector<TrackedObstacle*>&, std_msgs::Header&);

    void publish_allObs(const std::vector<MapElement*>&, std_msgs::Header&);


    void publishLabelHelper(const Eigen::Affine3d& pose, const std::string& label);


    bool isCamUpdated();
    void ResetCamFlag();
    bool updateParams();

    // individual obstacle merge
    obstacle_ind_merge *mpObstacle_ind_merge;
    // obstacles merge
    Obstacle_merge *mpObstacle_merge;

    // ros::NodeHandle nh;
    ros::Publisher wide_publisher, long_publisher, road_publisher;

    visualization_msgs::Marker mPoints;
    visualization_msgs::Marker mReferencePoints;
    visualization_msgs::Marker mKeyFrames;
    visualization_msgs::Marker mReferenceKeyFrames;
    visualization_msgs::Marker mCovisibilityGraph;
    visualization_msgs::Marker mMST;
    visualization_msgs::Marker mCurrentCamera;
    visualization_msgs::Marker mObsPoints;
    visualization_msgs::Marker mObsPoints_t;
    visualization_msgs::Marker mObsPoints_wide;
    visualization_msgs::Marker mObsPoints_wide_t;
    visualization_msgs::Marker mRoadArrow;
    visualization_msgs::Marker mRoadDisk;
    visualization_msgs::Marker mRoadArrow_t;
    std::vector<visualization_msgs::Marker> mRoad_dataset;
    std::vector<visualization_msgs::Marker> mRoad_disk_dataset;

    visualization_msgs::MarkerArray mObsBatch;
    visualization_msgs::MarkerArray mObsBatch_wide;
    visualization_msgs::MarkerArray mRoadArrows;

    float fCameraSize;
    float fPointSize;

    Eigen::Isometry3d new_pose;
    Eigen::Matrix3d rot_imu2cam;
    bool mbCameraUpdated;

    double road_pitch;
    double pitch_degree;
};



#endif //PROJECT_MAP_PUBLISHER_H
