//
// Created by hd on 1/10/18.
//
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "map_add.h"

using namespace std;
using namespace message_filters;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmerge_single_node");

    ros::NodeHandle n_private("~");
    ros::NodeHandle nh;

    int input_type;
    n_private.param<int>("input_type", input_type, 0);
    // obstacle individual merge
    obstacle_ind_merge *mpObstacle_ind_merge;

    // obstacles merge
    Obstacle_merge *mpObstacle_merge;

    MapPublisher *mpMapPublisher;

    mpObstacle_merge = new Obstacle_merge();

    mpObstacle_ind_merge = new obstacle_ind_merge(mpObstacle_merge);

    mpMapPublisher = new MapPublisher(nh, n_private, mpObstacle_ind_merge, mpObstacle_merge);

    MapAdd mapAdd(nh, mpObstacle_ind_merge, mpObstacle_merge, mpMapPublisher, input_type);

    double time_diff;

    string pose_topic, map_topic, map_topic_2;

//    pose_topic = "/ugv_slam_node/posestamped";
    pose_topic = "/gps_aligned/pose";
    map_topic = "/wide/map_msg";

    time_diff = 0.4;

    if (n_private.getParam("topic/pose", pose_topic)) {
        ROS_WARN("Pose Info Subscribing to: %s", pose_topic.c_str());
    }

    if (n_private.getParam("topic/map", map_topic)) {
        ROS_WARN("Cam Map Info Subscribing to: %s", map_topic.c_str());
    }

    if (n_private.getParam("topic/time_diff", time_diff)) {
        ROS_WARN("Time difference is set to: %f", time_diff);
    }

    ROS_WARN("now begin to subscribe to the map...");


    ///*********Subscribe to pose and 1 source of maps! ****************//

    cout << "now subscribing to one map..." << endl;

    Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, pose_topic, 20);

    Subscriber<obstacle_msgs::MapInfo> map_sub(nh, map_topic, 5);

    typedef sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, obstacle_msgs::MapInfo> SyncPolicy;

    Synchronizer<SyncPolicy> sync(SyncPolicy(10), pose_sub, map_sub);

    sync.setMaxIntervalDuration(ros::Duration(time_diff));

    sync.registerCallback(boost::bind(&MapAdd::addMapSingle, &mapAdd, _1, _2));



    ros::spin();

    return 0;
}
