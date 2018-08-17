//
// Created by hd on 1/11/18.
//
#include "map_publisher.h"
// C++
#include <string>
#include <vector>
#include <map>
#include <eigen_conversions/eigen_msg.h>
#include <opencv2/core/eigen.hpp>
#include <tf/tf.h>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 2)
{
    std::ostringstream out;
    out << std::setprecision(n) << a_value;
    return out.str();
}

MapPublisher::MapPublisher(ros::NodeHandle n, ros::NodeHandle n_private, obstacle_ind_merge *pObstacle_ind_merge, Obstacle_merge *pObstacle_merge):
        n_local_(n_private), mpObstacle_ind_merge(pObstacle_ind_merge), mpObstacle_merge(pObstacle_merge)
{
    const char* MAP_FRAME_ID = "world";
    const char* CAMERA_NAMESPACE = "cam";

    /***** obstacle map from static obstacle database (std::map<index_t, MapElement *> mapDatabase) ****/
    //region Description
    {

        mObsPoints.header.frame_id = MAP_FRAME_ID;
        mObsPoints.ns = "obstacles";
        mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
        mObsPoints.mesh_use_embedded_materials = true;
//        mObsPoints.type = visualization_msgs::Marker::CUBE;
//        mObsPoints.pose.orientation.w = 1.0;
        mObsPoints.action = visualization_msgs::Marker::ADD;
//        mObsPoints.color.a = 1.0f;

        mObsPoints_wide.header.frame_id = MAP_FRAME_ID;
        mObsPoints_wide.ns = "obstacles_wide";
        mObsPoints_wide.type = visualization_msgs::Marker::CUBE;
        mObsPoints_wide.pose.orientation.w = 1.0;
        mObsPoints_wide.action = visualization_msgs::Marker::ADD;
        mObsPoints_wide.color.a = 1.0f;

        /***** obstacle label helper ****/
        mObsPoints_t.header.frame_id = MAP_FRAME_ID;
        mObsPoints_t.ns = "obstacles_text";
        mObsPoints_t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mObsPoints_t.action = visualization_msgs::Marker::ADD;
        mObsPoints_t.scale.z = 1.8;
        mObsPoints_t.color.a = 1.0f;
        mObsPoints_wide_t.header.frame_id = MAP_FRAME_ID;
        mObsPoints_wide_t.ns = "obstacles_wide_text";
        mObsPoints_wide_t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mObsPoints_wide_t.action = visualization_msgs::Marker::ADD;
        mObsPoints_wide_t.scale.z = 1.3;
        mObsPoints_wide_t.color.a = 1.0f;
        /***** Road slope information ****/
        mRoadArrow.header.frame_id = MAP_FRAME_ID;
        mRoadArrow.ns = "road_slope_arrow";
        mRoadArrow.type = visualization_msgs::Marker::ARROW;
        mRoadArrow.action = visualization_msgs::Marker::ADD;
        mRoadArrow.color.a = 1.0f;
        mRoadArrow.color.r = 1.0f;
        mRoadArrow.color.g = 0.0f;
        mRoadArrow.color.b = 0.0f;
        mRoadArrow.scale.x = 0.2;
        mRoadArrow.scale.y = 0.5;
        mRoadArrow.scale.z = 0.6;
        mRoadArrow.pose.orientation.w = 1.0;

        mRoadDisk.header.frame_id = MAP_FRAME_ID;
        mRoadDisk.ns = "road_slope_disk";
        mRoadDisk.type = visualization_msgs::Marker::CUBE;
        mRoadDisk.action = visualization_msgs::Marker::ADD;
        mRoadDisk.scale.z = 3;
        mRoadDisk.scale.x = 3;
        mRoadDisk.scale.y = 0.01;
        mRoadDisk.color.a = 0.8f;
        mRoadDisk.color.r = 0.84f;
        mRoadDisk.color.g = 0.77f;
        mRoadDisk.color.b = 0.13f;
        mRoadDisk.pose.orientation.w = 1.0;


        mRoadArrow_t.header.frame_id = MAP_FRAME_ID;
        mRoadArrow_t.ns = "road_slope_text";
        mRoadArrow_t.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mRoadArrow_t.action = visualization_msgs::Marker::ADD;
        mRoadArrow_t.scale.z = 1.8;
        mRoadArrow_t.scale.x = 1.8;
        mRoadArrow_t.scale.y = 1.8;
        mRoadArrow_t.color.a = 1.0f;
        mRoadArrow_t.color.r = 1.0f;

    }
    //endregion

//    Configure Current Camera
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id = 4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x = 0.2;//0.2; 0.03
    mCurrentCamera.pose.orientation.w = 1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g = 1.0f;
    mCurrentCamera.color.a = 1.0f;
//    rot_imu2cam << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    //Configure Publisher
    wide_publisher = n.advertise<visualization_msgs::MarkerArray>("wide_mapObs", 10);
    long_publisher = n.advertise<visualization_msgs::MarkerArray>("long_mapObs", 10);
    road_publisher = n.advertise<visualization_msgs::MarkerArray>("road_slope", 10);
//    timer_ = n.createTimer(ros::Duration(1.0), &Obstacle_merge::timerCallback, mpObstacle_merge, false, false);

//    updateParams();
}

void MapPublisher::Refresh(std_msgs::Header &header)
{
    if(isCamUpdated())
    {
        Eigen::Isometry3d Tc2w = GetCurrentCameraPose();
        PublishCurrentCamera(Tc2w, header);
        ResetCamFlag();
    }
    /////******This block is for old full obstacle publish ******//

    if(mpObstacle_ind_merge->isObsUpdated()){

        std::map<unsigned long int, MapElement* > mDetPoints = mpObstacle_merge->mapDatabase_long;

        PublishObs_long(mDetPoints, header);

        std::map<unsigned long int, MapElement* > mDetPoints_wide = mpObstacle_merge->mapDatabase;

        PublishObs_wide(mDetPoints_wide, header);

        mpObstacle_ind_merge->ResetObsUpdated();
    }
//    std::vector<double> mRoad_set = mpObstacle_merge->roadSlopeSet;

    if(mpObstacle_merge->frame_num % 10 == 1)
        Publish_road_arrow(road_pitch, header);
    /////******This block is for experimental use, only wide obstacles ******//

//
//    if(mpObstacle_ind_merge->isObsUpdated()){
//
//        std::vector<MapElement* > mDetPoints;
//
//        for (const auto &s : mpObstacle_merge->mapDatabase){
//
//            mDetPoints.push_back(s.second);
//        }
//
//        publish_allObs(mDetPoints, header);
//
//        mpObstacle_ind_merge->ResetObsUpdated();
//
//    }

    /////******This block is for tracked obstacle (wide), but currently cannot be used ******//

//    if(mpObstacle_merge->isObsUpdated()) {
//
//        std::vector<TrackedObstacle*> mTracked_Obs = mpObstacle_merge->tracked_obstacles_;
//
//        publish_trackedObs(mTracked_Obs, header);
//
//
//        mpObstacle_merge->ResetObsUpdated();
//    }
//******This block is for tracked obstacle (wide), but currently cannot be use******//
}

void MapPublisher::PublishObs_long(const std::map<unsigned long int, MapElement* > &mObs, std_msgs::Header &header){

    mObsBatch.markers.clear();

    std::map<unsigned long, MapElement* >::const_iterator it;

    for(it = mObs.begin(); it!=mObs.end(); it++) {

        if(((*it).second->category!="traffic light")&&((*it).second->category!="traffic sign")){

            mObsPoints.id = (int) (*it).first;

//            if((*it).second->temp_diff < 70){

//                mObsPoints.color.r = 0.0f;
//                mObsPoints.color.b = 1.0f;
//                mObsPoints.color.g = 0.0f;
//            } else {
//                mObsPoints.color.g = 0.0f;
//                mObsPoints.color.r = 1.0f;
//                mObsPoints.color.b = 0.1f;
//            }

            Eigen::Quaterniond rot(AngleAxisd(M_PI_2, Vector3d::UnitX()) * AngleAxisd( M_PI, Vector3d::UnitZ()));

            if((*it).second->isTracked){
                rot = new_pose.matrix().topLeftCorner<3,3>() * rot;
                rot.normalize();

                mObsPoints.pose.orientation.x = rot.x();
                mObsPoints.pose.orientation.y = rot.y();
                mObsPoints.pose.orientation.z = rot.z();
                mObsPoints.pose.orientation.w = rot.w();

                (*it).second->world_orient = rot;
                (*it).second->isTracked = false;
            }
            else{

                mObsPoints.pose.orientation.x = (*it).second->world_orient.x();
                mObsPoints.pose.orientation.y = (*it).second->world_orient.y();
                mObsPoints.pose.orientation.z = (*it).second->world_orient.z();
                mObsPoints.pose.orientation.w = (*it).second->world_orient.w();
            }


            mObsPoints.pose.position.x = (*it).second->center_point_world.x();
            mObsPoints.pose.position.y = (*it).second->center_point_world.y();
            mObsPoints.pose.position.z = (*it).second->center_point_world.z();

            if((*it).second->category == "car"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Car_Model/Car.dae";
                mObsPoints.scale.x = 0.5;
                mObsPoints.scale.y = 0.5;
                mObsPoints.scale.z = 0.5;
//                    mObsPoints.scale.x = 1.5;
//                    mObsPoints.scale.y = 1.2;
//                    mObsPoints.scale.z = 3;

            }else if((*it).second->category == "bus"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Protect_Van/Protect_Van.dae";
                mObsPoints.scale.x = 0.5;
                mObsPoints.scale.y = 0.5;
                mObsPoints.scale.z = 0.5;
//                    mObsPoints.scale.x = 2.5;
//                    mObsPoints.scale.y = 2.3;
//                    mObsPoints.scale.z = 5;

            }else if((*it).second->category == "bike"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Bicycle/Bicycle.dae";
//                mObsPoints.scale.x = 0.4;
//                mObsPoints.scale.y = 1.6;
//                mObsPoints.scale.z = 1.6;
                mObsPoints.scale.x = 0.4;
                mObsPoints.scale.y = 0.4;
                mObsPoints.scale.z = 0.4;

            }else if((*it).second->category == "motor"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Bike_01/Bike_01.dae";
//                mObsPoints.scale.x = 0.4;
//                mObsPoints.scale.y = 1.6;
//                mObsPoints.scale.z = 1.6;
                mObsPoints.scale.x = 1;
                mObsPoints.scale.y = 1;
                mObsPoints.scale.z = 1;

            }else if((*it).second->category == "person"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Body/Bodymesh.dae";
                mObsPoints.scale.x = 0.4;
                mObsPoints.scale.y = 0.4;
                mObsPoints.scale.z = 0.4;
//                    mObsPoints.scale.x = 0.3;
//                    mObsPoints.scale.y = 1.7;
//                    mObsPoints.scale.z = 0.3;
//                    mObsPoints.color.g = 0.0f;
//                    mObsPoints.color.r = 1.0f;
//                    mObsPoints.color.b = 0.1f;
            }else if((*it).second->category == "truck"){
                mObsPoints.type = visualization_msgs::Marker::MESH_RESOURCE;
                mObsPoints.mesh_resource = "package://cmerge/mesh/Ambulance/Ambulance.dae";
                mObsPoints.scale.x = 0.3;
                mObsPoints.scale.y = 0.3;
                mObsPoints.scale.z = 0.3;
//                    mObsPoints.scale.x = 2.7;
//                    mObsPoints.scale.y = 2;
//                    mObsPoints.scale.z = 3.5;
            }else {
                mObsPoints.type = visualization_msgs::Marker::CUBE;
                mObsPoints.scale.x = (*it).second->width;
                mObsPoints.scale.y = (*it).second->height;
                mObsPoints.scale.z = (*it).second->width;
            }
            mObsPoints.header.stamp = header.stamp;

            mObsBatch.markers.push_back(mObsPoints);

//            mObsPoints_t.id = (int) (*it).first;
//            mObsPoints_t.pose.position.x = (*it).second->center_point_world.x()
//                                           + (*it).second->width * 2;
//            mObsPoints_t.pose.position.y = (*it).second->center_point_world.y()
//                                           + (*it).second->height * 4;
//            mObsPoints_t.pose.position.z = (*it).second->center_point_world.z() + 2;
//            mObsPoints_t.text = (*it).second->category;
//                if(fabs((*it).second->velocity.z()) > 0.1)
//                    mObsPoints_t.text += ":  " + to_string_with_precision((*it).second->temp_diff);
//                mObsPoints_t.text =  std::to_string((*it).first)
//                                    + " | " + to_string_with_precision((*it).second->center_point_local.z());
//            mObsPoints_t.text += (*it).second;
//            mObsPoints_t.color.r = 1.0f;
//            mObsPoints_t.header.stamp = header.stamp;
//            mObsBatch.markers.push_back(mObsPoints_t);
//            }
        }
    }

    long_publisher.publish(mObsBatch);

}

void MapPublisher::PublishObs_wide(const std::map<unsigned long int, MapElement* > &mObs, std_msgs::Header &header){

    mObsBatch_wide.markers.clear();

    std::map<unsigned long, MapElement* >::const_iterator it;

    for(it = mObs.begin(); it!=mObs.end(); it++) {

//        if( ((*it).second->width > 1.4) && ((*it).second->height > 1.4) ){

            mObsPoints_wide.id = (int) (*it).first;

                mObsPoints_wide.color.r = 0.0f;
                mObsPoints_wide.color.b = 0.0f;
                mObsPoints_wide.color.g = 1.0f;

                mObsPoints_wide.pose.position.x = (*it).second->center_point_world.x();
                mObsPoints_wide.pose.position.y = (*it).second->center_point_world.y();
                mObsPoints_wide.pose.position.z = (*it).second->center_point_world.z()+1;
                mObsPoints_wide.scale.x = (*it).second->width/2;
                mObsPoints_wide.scale.y = (*it).second->height/2;
                mObsPoints_wide.scale.z = (*it).second->width/2;
                mObsPoints_wide.header.stamp = header.stamp;

                mObsBatch_wide.markers.push_back(mObsPoints_wide);

                mObsPoints_wide_t.id = (int) (*it).first;
                mObsPoints_wide_t.pose.position.x = (*it).second->center_point_world.x()
                                               + (*it).second->width * 2;
                mObsPoints_wide_t.pose.position.y = (*it).second->center_point_world.y()
                                               + (*it).second->height * 2;
                mObsPoints_wide_t.pose.position.z = (*it).second->center_point_world.z() + 1;

                mObsPoints_wide_t.text = std::to_string((*it).first) // + ": " + to_string_with_precision((*it).second->id);
                                    + " | " + to_string_with_precision((*it).second->center_point_local.z());
//            mObsPoints_wide_t.text += (*it).second;
//                mObsPoints_wide_t.color.b = 1.0f;
//                mObsPoints_wide_t.header.stamp = header.stamp;
//                mObsBatch_wide.markers.push_back(mObsPoints_wide_t);
//            }
//        }
    }

    wide_publisher.publish(mObsBatch_wide);
}

void MapPublisher::Publish_road_arrow(double pitch, std_msgs::Header &header) {

    mRoadArrows.markers.clear();
    mRoadArrow.points.clear();

    std::vector<visualization_msgs::Marker>::const_iterator it, it2;

    for (it = mRoad_dataset.begin(); it!=mRoad_dataset.end(); it++) {

        mRoadArrow = (*it);
        mRoadArrow.id = it - mRoad_dataset.begin();
        mRoadArrow.header.stamp = header.stamp;

        mRoadArrows.markers.push_back(mRoadArrow);
    }

    for (it2 = mRoad_disk_dataset.begin(); it2!=mRoad_disk_dataset.end(); it2++) {

        mRoadDisk = (*it2);
        mRoadDisk.id = it2 - mRoad_disk_dataset.begin();
        mRoadDisk.header.stamp = header.stamp;

        mRoadArrows.markers.push_back(mRoadDisk);
    }

    mRoadArrow.points.clear();
    mRoadArrow.id +=1;
    mRoadArrow.header.stamp = header.stamp;

    mRoadDisk.id +=1;
    mRoadDisk.header.stamp = header.stamp;

    Eigen::Isometry3d disk_to_cam = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond rot(AngleAxisd(pitch, Vector3d::UnitX()));
//    Eigen::Quaterniond orientation_(new_pose.matrix().topLeftCorner<3,3>());

//    orientation_ = orientation_ * rot;
    Eigen::Vector3d arrow_pose1, arrow_pose2, arrow_pose_t;
    Eigen::Vector3d start_local (0, 0, 14);
    Eigen::Vector3d end_local (0, -2, 14);
    Eigen::Vector3d text_local (0, 1, 14);
    disk_to_cam.linear() = rot.toRotationMatrix();
    disk_to_cam.translation() = start_local;

    start_local = rot * start_local;
    end_local = rot * end_local;
    text_local = rot * text_local;

    arrow_pose1 = new_pose * start_local;   // arrow start point position
    arrow_pose2 = new_pose * end_local;     // arrow end point position
    arrow_pose_t = new_pose * text_local;   // arrow label helper position
    disk_to_cam = new_pose * disk_to_cam;

    geometry_msgs::Point start_p_, end_p_;
    start_p_.x = arrow_pose1.x();
    start_p_.y = arrow_pose1.y();
    start_p_.z = arrow_pose1.z();

    end_p_.x = arrow_pose2.x();
    end_p_.y = arrow_pose2.y();
    end_p_.z = arrow_pose2.z();
    mRoadArrow.points.push_back(start_p_);
    mRoadArrow.points.push_back(end_p_);

    mRoadArrow_t.pose.position.x = arrow_pose_t.x();
    mRoadArrow_t.pose.position.y = arrow_pose_t.y();
    mRoadArrow_t.pose.position.z = arrow_pose_t.z();
    pitch_degree = pitch*180.0/M_PI;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3) << pitch_degree;
    std::string pitch_string = ss.str();
    mRoadArrow_t.text = "pitch: " + pitch_string;
//
//    mRoadDisk.pose.position.x = arrow_pose1.x();
//    mRoadDisk.pose.position.y = arrow_pose1.y();
//    mRoadDisk.pose.position.z = arrow_pose1.z();

    tf::poseEigenToMsg(disk_to_cam, mRoadDisk.pose);
    mRoad_dataset.push_back(mRoadArrow);
    mRoadArrows.markers.push_back(mRoadArrow);
    mRoadArrows.markers.push_back(mRoadDisk);
    mRoadArrows.markers.push_back(mRoadArrow_t);

    road_publisher.publish(mRoadArrows);

}

void MapPublisher::PublishCurrentCamera(const Eigen::Isometry3d &Tcw,std_msgs::Header &header)
{
    mCurrentCamera.points.clear();

    float d = 0.5;

    //Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) << 0, 0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) << d, d*0.8, d*0.5, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) << d, -d*0.8, d*0.5, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -d, -d*0.8, d*0.5, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -d, d*0.8, d*0.5, 1);

    Eigen::Matrix3d R = Tcw.matrix().topLeftCorner<3,3>();
    Eigen::Vector3d t = Tcw.translation();
    cv::Mat Tcw_ = cv::Mat::eye(4,4, CV_32F);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Tcw_.at<float>(i, j) = R(i, j);
        }
    }
    for (int i = 0; i < 3; i++) {
        Tcw_.at<float>(i, 3) = t(i);
    }

    cv::Mat Twc = Tcw_.inv();
    cv::Mat ow = Twc*o;
    cv::Mat p1w = Twc*p1;
    cv::Mat p2w = Twc*p2;
    cv::Mat p3w = Twc*p3;
    cv::Mat p4w = Twc*p4;

    geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

    mCurrentCamera.header.stamp = header.stamp;

    wide_publisher.publish(mCurrentCamera);
}

void MapPublisher::publish_allObs(const std::vector<MapElement*>& obs_vector, std_msgs::Header& header){

//    visual_tools_->deleteAllMarkers();
//
//    std::vector<MapElement*>::const_iterator it;
//
//    std::cout << "tracked num: " << obs_vector.size() << std::endl;
//
//    for(it = obs_vector.begin(); it!=obs_vector.end(); it++) {
//
//        const MapElement* single_obs = *it;
//
//        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
//
//        pose.translation() = single_obs->center_point_world;
//
//        visual_tools_->publishWireframeCuboid(pose, single_obs->width, single_obs->width,
//                                              single_obs->height, rvt::colors::MAGENTA, "wide_obstacles");
//
////        publishLabelHelper(pose, "")
//    }
//
//    visual_tools_->trigger();


    mObsBatch_wide.markers.clear();

    std::vector<MapElement*>::const_iterator it;

    for(it = obs_vector.begin(); it!=obs_vector.end(); it++) {

        if (0.4 < (*it)->width) {
            if (0.4 < (*it)->height) {
                mObsPoints_wide.id = (int) ((*it)->id);
                mObsPoints_wide.color.r = 0.0f;
                mObsPoints_wide.color.b = 1.0f;
                mObsPoints_wide.color.g = 0.0f;
                mObsPoints_wide.pose.position.x = (*it)->center_point_world.x();
                mObsPoints_wide.pose.position.y = (*it)->center_point_world.y();
                mObsPoints_wide.pose.position.z = (*it)->center_point_world.z();
                mObsPoints_wide.scale.x = (*it)->width;
                mObsPoints_wide.scale.y = (*it)->height;
                mObsPoints_wide.scale.z = (*it)->width;
                mObsPoints_wide.header.stamp = header.stamp;
                mObsBatch_wide.markers.push_back(mObsPoints_wide);
                mObsPoints_wide_t.id = (int) (*it)->id;
                mObsPoints_wide_t.pose.position.x = (*it)->center_point_world.x()
                                                + (*it)->width * 2;
                mObsPoints_wide_t.pose.position.y = (*it)->center_point_world.y()
                                                + (*it)->height * 2;
                mObsPoints_wide_t.pose.position.z = (*it)->center_point_world.z() + 1;
                mObsPoints_wide_t.text = (*it)->category + ": " + to_string_with_precision((*it)->width);
                mObsPoints_wide_t.color.r = 1.0f;
                mObsPoints_wide_t.header.stamp = header.stamp;
                mObsBatch_wide.markers.push_back(mObsPoints_wide_t);
            }
        }
    }

    wide_publisher.publish(mObsBatch_wide);

}

void MapPublisher::SetCurrentCameraPose(const Eigen::Isometry3d &Tc2w)
{
    new_pose = Tc2w;
    mbCameraUpdated = true;
}

Eigen::Isometry3d MapPublisher::GetCurrentCameraPose()
{
    return new_pose;
}

bool MapPublisher::isCamUpdated()
{
    return mbCameraUpdated;
}

void MapPublisher::ResetCamFlag()
{
    mbCameraUpdated = false;
}

void MapPublisher::SetCurrentRoadAngle(double pitch){

    road_pitch = pitch;
}

