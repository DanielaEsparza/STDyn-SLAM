# STDyn-SLAM
STDyn-SLAM: A Stereo Vision and Semantic Segmentation Approach for SLAM in Dynamic Outdoor Environments

**Authors:** Daniela Esparza and Gerardo Flores.

# License
STDyn-SLAM released under a [GPLv3 license](https://github.com/DanielaEsparza/STDyn-SLAM/blob/master/LICENSE)

# Run

Before executing, please run the next in the terminal. Modify ROOT_PATH by your container folder of STDyn-SLAM.
```
export  ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:ROOT_PATH/STDyn-SLAM/Examples/ROS/ORB_SLAM2_PointMap_SegNetM
```

## Real-Time Stereo

Execute the ros package os the stereo camera, modify the [STEREO_RealTime.launch](https://github.com/DanielaEsparza/STDyn-SLAM/blob/master/STEREO_RealTime.launch)

```
cd src/STDyn-SLAM
roslaunch STEREO_RealTime.launch
```

## KITTI Dataset

