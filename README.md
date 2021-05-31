
# Folder structure
Note: Under this project is the software following software used:


*  PX4(version v1.11.3)
*  QGroundControl
*  ROS Noetic
*  Gazebo11

## spoof_gps_plugin_files
This is the folder where the gazebo plugin for the GPS module is change to be able simulate of spoofing the GPS signal.
Where there are publish and subscriber topics from ROS to initiate the spoofing. 

* The CMakeLists.txt file shall be placed at PX4-Autopilot/Tools/sitl_gazebo
* The gazebo_gps_plugin.cpp shall be placed at PX4-Autopilot/Tools/sitl_gazebo/src
* The gazebo_gps_plugin.h shall be placed at PX4-Autopilot/Tools/sitl_gazebo/include



## Work_sim_spoof
This is for simuluation spoofing attack on a drone in gazebo0 with SITL PX4(v1.11.3)

## Thesis
This is for the thesis where the job is to detect spoofing
