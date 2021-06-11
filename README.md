# Aceinna OpenRTK ROS Driver

Overview
--------
This is the ROS driver for Aceinna OpenRTK series GNSS/INS integrated navigation module products, with support for **Serial (UART)** port and **Ethernet** port. 

The ROS driver source files are located in the subfolder "openrtk_ros".

Several primary messages output by OpenRTK series products are defined in the subfolder "openrtk_msg", which are composed of ROS "std_msg" header and proprietary OpenRTK message content:

- openrtk_gnss.msg: GNSS solution message
- openrtk_imu.msg: raw IMU data message
- openrtk_ins.msg: INS solution message

  Note: User could modify the content of the messages that are defined in the openrtk_msg/msg folder, and the modified messages should comply with ROS definitions.



Usage
--------

### Prerequisites

- PC with Ubuntu 18.04
- Install ROS Melodic, please refer to http://wiki.ros.org/melodic/Installation/Ubuntu
- OpenRTK330LI EVK



### Build

The following are steps to build the ROS driver from source code in your local development environment:

1. Go to your ROS workspace

   ​	`cd ~/catkin_ws`   

2. Copy folders to your ROS workspace 

   ​	`cp openrtk_ros openrtk_msg ./src`

3. Compile the code

   ​	`catkin_make`

**Note**:   This driver includes support for serial port and Ethernet. The messages output by the two ports are the same. It is recommended to use only one.   To select an output, you only need to select which thread to create at line 122 of /ros_rtk/src/driver/driver.cpp.    

When you choose the Ethernet output method, you need to complete the following two things:   
1) Modify line 24 of driver.cpp according to the IP of your ROS system
2) Modify the OpenRTK lower computer code, cancel NetBios to obtain the host IP, change it to static IP, and ensure that the IP and the ROS system IP are in the same network segment. 

### Operation

1. Launch the node

   ​	`roslaunch openrtk_ros run.launch`	

2. List topics

   ​	`rostopic list`

3. Message echo (xxx = imu, ins, gnss)

   ​	`rostopic echo /rtk/topic_rtk_xxx`

   ​	                

## License

The source code is licensed under the MIT license --- refer to the LICENSE file for details.

