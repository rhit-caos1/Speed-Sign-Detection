# Autonomous Train Control Robot

## Video


https://user-images.githubusercontent.com/80802375/226062145-78f7c51c-8b28-4bd6-8cc3-790ab461a7e0.mp4



## Overview
This ROS2 humble package combines yolo object detection, openCV LCD detection, and two PincherX 100 robot arms to achieve automonus Caltrain MP36PHI train control in train simulator Train Sim World Peninsula Corridorroute DLC.

## Prerequisites
This package depends on several pacakges. In order to get all the functions, please install this package: https://github.com/m-elwin/numsr_patches.

Follow the installation section in yolov7 repo to install required environment: https://github.com/WongKinYiu/yolov7#installation. Also, check OpenCV is installed.

Also, yolo v7 is required for this. Due to the ROS2 structure, yolo v7 should be installed as a python package. The files which required for yolo v7 package installation is provide in the `yolo_package` folder. Run `pip install yolo_pkg` in `yolo_package` folder to install

After build the package, copy the weight `best.pt` in `result/v8` folder to the workspace. Then copy the `models` and `utils` folder in `yolo_package/src/yolo_pkg/` to `install/lib/detection/lib/detection` in the workspace. These two steps are important for pytorch to run correctly in ros2 node.

## Hardwares:
1. one web cam for speed reading (Realsense rgb camera is used as a webcam in this project)
2. one graphic capture card for video transmission
3. two PincherX 100 robot arms
4. Laser cutted PincherX fixture

## Start The Code

In this package, there are two launch files. One is for precption and control nodes, one is for robot control nodes. This gives the user ability to kill robot control when robot is not working properly, and keep detection and control work, which can still provide instuctions for humans to operate train. 

The order of launching the launch files are important, but user can still use services to resume robot control with different launching order.

First, to launch robot control nodes, start the xarm launch file:

`ros2 launch control xsarm_dual.launch.xml`

Then launch precption and control nodes:

`ros2 launch control launch_control.xml`

The contol system should start to drive the train by commanding two robot arm to move the levers.
If the train stopped at one station and user would like to move the train again, use following command:

`ros2 service call /resume std_srvs/srv/Empty`

To override the latest command and take over control before the next control command, use the following command to move two robot arm to "sleep" position. The robot will take over the control after receiving a new command.

`ros2 service call /ID std_srvs/srv/Empty`


## Limitations

This program relies on the object detection and speed recognition system. If those two systems return incorrect readings, it will cause the controller behave incorrectly.



