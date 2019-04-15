# fjj_team_project
Final team project of "Special topics in Robotics" Class

2018-1, SKKU in Korea

## Packages
Package and node description

### fjj_bot
| Data | Publisher | Topic Name | Type |
| ------------- | ------------- | ------------- | ------------- |
| RGB image  | vrep | /image/hand_depth | sensor_msgs/Image |
| Depth image | vrep | /image/hand_color | sensor_msgs/Image |
| Lidar scan | vrep | /lidar/front_scan | sensor_msgs/LaserScan |
| Grasping cmd | main.py | /robot/closing | std_msgs/Bool |
| Robot Pose2d | slam.py | /robot/pose2d | geometry_msgs/Pose2D |
| Wheel velocity | (not published) |  | remoteApi |
| Control robot arm | (not published) |  | remoteApi |

## Requirments
necessary
- ros
- vrep
- python3

optional
- tensorflow-gpu(with CUDA)
- darknet yolo (GNU GPLv3)
    ```
    https://github.com/thtrieu/darkflow
    ```
- gmapping SLAM
    ```
    http://wiki.ros.org/Build%20a%20map%20with%20SLAM
    ```

## Build
1. Build your workspace
    ```
    cd ros_ws
    catkin_make
    ```


## Run
1. roscore
1. vrep
1. Run your launch file
    ```
    roslaunch fjj_bot run.launch
    ```

## Members
* FRANCISCO YUMBLA
* Hong-ryul Jung
* Woo Seok Jeong

