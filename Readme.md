## Stereo vision based obstacle mapping

** Don't star, this is not my active repo!** Currently it is under a private repository.

Original plan is to merge the two inputs of obstacle bounding boxes and generate a optimized obstacle cubicle map.

First try to reproduce the result of paper: `Stereo Vision-based Semantic 3D Object and Ego-motion Tracking for Autonomous Driving` of ECCV 2018.

```
@article{li2018stereo,
  title={Stereo Vision-based Semantic 3D Object and Ego-motion Tracking for Autonomous Driving},
  author={Li, Peiliang and Qin, Tong and Shen, Shaojie},
  journal={arXiv preprint arXiv:1807.02062},
  year={2018}
}
```

The `semantic_mapping_node` subscribes the obstacle map information **obstacle_detection::MapInfo** from two sets of cameras
and pose information **geometry_msgs::PoseStamped /ugv_slam_node/posestamped** from wide camera.

### Prerequisite

The package is based on Ubuntu 16.04 with ROS Kinetics. Additionally you should install:
1. OPENCV 3.1+
2. Eigen 3.2+
3. [Ceres Solver](http://ceres-solver.org/installation.html)

4. Install [VSLAM](https://github.com/lrse/sptam.git) and [object detection](git@github.com:zhanghanduo/cubicle_detect.git) repositories in <YOUR_CATKIN_WORKSPACE>:


### Installation
```
    cd catkin_ws/src

    git clone git@github.com:zhanghanduo/stereo_semantic_mapping.git

    cd ..

    catkin_make
```

### Demo

```
    roslaunch semantic_mapping demo.launch

```


### RoadMap

- [] Early Development - Mapping without optimization
- [] Object Tracking
- [] Sparse Feature Observation
- [] Semantic 3D Object Measurement
- [] Point Cloud Alignment
- [] Joint Optimization
- [] Evaluation
