# STDyn-SLAM
STDyn-SLAM: A Stereo Vision and Semantic Segmentation Approach for SLAM in Dynamic Outdoor Environments

**Authors:** Daniela Esparza and Gerardo Flores.

# License
STDyn-SLAM released under a [GPLv3 license](https://github.com/DanielaEsparza/STDyn-SLAM/blob/master/LICENSE)


# Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/3tnkwvRnUss/0.jpg)](https://www.youtube.com/watch?v=3tnkwvRnUss)


# Install

## Prerequisites

The necessary prerequisites are found in [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) and DS-SLAM(https://github.com/ivipsourcecode/DS-SLAM):
- Pangolin
- OpenCV
- Eigen3
- DBoW2 and g2o
- ROS

- SegNet
- OctoMap and RVIZ

### SegNet
Download and compile the SegNet package in /Examples/ROS/ORB_SLAM2_PointMap_SegNetM (https://github.com/TimoSaemann/caffe-segnet-cudnn5).

You also can download a new version of SegNet from (https://github.com/navganti/caffe-segnet-cudnn7). But you have to modify the root of SegNet in the CmakeLists.txt from /STDyn-SLAM, STDyn-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM/ and STDyn-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM/libsegmentation

### OctoMap
You have to create a catkin workspace and download the OctoMap in /src

## Building STDyn-SLAM

Change the name of the catkin workspace to yours.

```
cd catkin_ws/src
git clone https://github.com/DanielaEsparza/STDyn-SLAM
cd STDyn-SLAM
chmod +x BUILD.sh
./BUILD.sh
```

# Run

Before executing, please run the next in the terminal. Modify ROOT_PATH by your container folder of STDyn-SLAM.
```
export  ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:ROOT_PATH/STDyn-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM
```

## Real-Time Stereo

Execute the ros package of the stereo camera, modify the [STEREO_RealTime.launch](https://github.com/DanielaEsparza/STDyn-SLAM/blob/master/STEREO_RealTime.launch)

```
cd src/STDyn-SLAM
roslaunch STEREO_RealTime.launch
```

## KITTI Dataset

Modify the KITTI Dataset path in the KITTI.launch

```
cd src/STDyn-SLAM
roslaunch KITTI.launch
```

## Own Sequence

Modify the path of your sequence in the PATH.launch

```
cd src/STDyn-SLAM
roslaunch PATH.launch
```
