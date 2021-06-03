# ROS_Driver-for-ACEINNA-RTK

Overview
--------
- This is a C++ [ROS]driver for Aceinnna Rtk (IMU/GNSS/INS) Receivers. 
- Supports serial

Usage
--------
The `master` branch for this driver functions on ROS Melodic, if you don't know how to install , you can reffer to http://wiki.ros.org/melodic/Installation/Ubuntu;

---Packages
-----'rtk_msg'
     rtk_msg contants three msgs, rtkmsg_gnss、rtkmsg_imu、rtkmsg_ins, each supports log Aceinna-RTK GNSS/IMU/INS message.
-----'ros_rtk'
     ros_rtk is the ros driver for Aceinna-RTK, it will process GNSS/IMU/INS message received form Aceinna-RTK by serial.
     log the message to ROS by Rostopic.

---How to use

cd ~/catkin_ws                    -----into your ros workspace
cp ros_rtk rtk_msg ./src          -----copy folders ros_rtk rtk_msg into folder src,which is under your ros workspace
catkin_make                       -----build
roslaunch ros_rtk run.launch      -----launch node ros_rtk , you check node name in ros_rtk/launch/run.launch
rostopic list                     -----list topics, you can see  /rtk/topic_rtk_gnss  /rtk/topic_rtk_ins /rtk/topic_rtk_imu
rostopic echo /rtk/topic_rtk_xxx  -----echo message for IMU/GNSS/INS
