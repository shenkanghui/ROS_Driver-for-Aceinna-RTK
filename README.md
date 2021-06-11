# ROS_Driver-for-ACEINNA-RTK

Overview
--------
- This is a C++ [ROS]driver for Aceinnna Rtk (IMU/GNSS/INS) Receivers. 
- Supports serial
- Supports Ethernet

Usage
--------
The `master` branch for this driver functions on ROS Melodic, if you don't know how to install , you can reffer to http://wiki.ros.org/melodic/Installation/Ubuntu;   

---Packages   
-----'rtk_msg'   
     rtk_msg contants three msgs, rtkmsg_gnss、rtkmsg_imu、rtkmsg_ins, each supports log Aceinna-RTK GNSS/IMU/INS message.   
     These three msgs are composed of ros std_msg header and specific rtk message content. You can use cmd "rosmsg show" to see the detial of each rtk_msg.   
-----'ros_rtk'   
     ros_rtk is the ros driver for Aceinna-RTK, it will process GNSS/IMU/INS message received form Aceinna-RTK by serial or eth and      
     log the message to ROS by Rostopic.   

---How to use

cd ~/catkin_ws                    -----into your ros workspace    
cp ros_rtk rtk_msg ./src          -----copy folders ros_rtk rtk_msg into folder src,which is under your ros workspace    
catkin_make                       -----build    
roslaunch ros_rtk run.launch      -----launch node ros_rtk , you check node name in ros_rtk/launch/run.launch     
rostopic list                     -----list topics, you can see  /rtk/topic_rtk_gnss  /rtk/topic_rtk_ins /rtk/topic_rtk_imu    
rostopic echo /rtk/topic_rtk_xxx  -----echo message for IMU/GNSS/INS

---Notice!!!
This driver includes support for serial port and Ethernet. The messages output by the two ports are the same. It is recommended to use only one.   
To select an output, you only need to select which thread to create at line 122 of /ros_rtk/src/driver/driver.cpp.    

When you choose the Ethernet output method, you need to complete the following two things:
1) Modify line 24 of driver.cpp according to the IP of your ROS system
2) Modify the OpenRTK lower computer code, cancel NetBios to obtain the host IP, change it to static IP, and ensure that the IP and the ROS system IP are in the same network segment. 

