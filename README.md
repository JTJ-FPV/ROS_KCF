# ROS_KCF
本项目将KCF算法整合到ROS中，使用者可以自行修改构建。
该KCF算法为João Faro开发，详情请见url：https://github.com/joaofaro/KCFcpp

开发环境为 Ubuntu18.04LTS ros-melodic opencv-3.4.10

编译：
cd ROS_KCF
catkin_make

该功能包下有四个节点
ROS_KCF_node usb_cam_node camera_kcf_node kcf_sub

ROS_KCF_node 对应的CPP文件为 Net_KCF.cpp 订阅的话题为 /camera/D435i/color/image_raw/compressed 
该话题是 realsense2_camera 功能包下的 rs_camera.launch 发布
运行 ROS_KCF_node:
首先运行 rs_camera.launch
roslaunch realsense2_camera rs_camera.launch
其次再运行 ROS_KCF_node
cd ROS_KCF
source ./devel/setup.bash
rosrun ROS_KCF ROS_KCF_node
在窗口通过鼠标框住需要跟踪的区域

usb_cam_node 对应的CPP文件为 usb_cam.cpp 订阅的话题为 /usb_cam/image_raw/compressed 
该话题是 usb_cam 功能包下的 usb_cam-test.launch 发布
运行 camera_kcf_node
首先运行 usb_cam-test.launch
roslaunch usb_cam usb_cam-test.launch
其次再运行 usb_cam_node
cd ROS_KCF
source ./devel/setup.bash
rosrun ROS_KCF usb_cam_node
在窗口通过鼠标框住需要跟踪的区域


camera_kcf_node 对应的CPP文件为 camera_kcf.cpp 该节点使用的是usb摄像头，需要查看 /dev/video0 是否存在
运行 camera_kcf_node
首先运行 roscore
其次再运行 camera_kcf_node
cd ROS_KCF
source ./devel/setup.bash
rosrun ROS_KCF camera_kcf_node
在窗口通过鼠标框住需要跟踪的区域


kcf_sub 对应的CPP文件为 kcf_sub.cpp 该节点是用来订阅追踪后的图片
运行kcf_sub
首先运行 roscore
其次再运行 kcf_sub
cd ROS_KCF
source ./devel/setup.bash

若想订阅 ROS_KCF_node 发布的追踪图片（该节点使用的摄像头设备为D435i）
rosrun ROS_KCF kcf_sub /D435i/Tracker/camera/image

若想订阅 usb_cam_node 发布的追踪图片
rosrun ROS_KCF kcf_sub /usb_cam/Tracker/camera/image

若想订阅 camera_kcf_node 发布的追踪图片
rosrun ROS_KCF kcf_sub /call_camera/Tracker/camera/image