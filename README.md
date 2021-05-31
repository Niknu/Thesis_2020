
# Folder structure
Note: Under this project is the software following software used:

*  PX4(version v1.11.3)
*  QGroundControl
*  ROS Noetic
*  Gazebo11
*  Scipy

## spoof_gps_plugin_files
This is the folder where the gazebo plugin for the GPS module is change to be able simulate of spoofing the GPS signal.
Where there are publish and subscriber topics from ROS to initiate the spoofing. 

* The **CMakeLists.txt** file shall be placed at **PX4-Autopilot/Tools/sitl_gazebo**
* The **gazebo_gps_plugin.cpp** shall be placed at **PX4-Autopilot/Tools/sitl_gazebo/src**
* The **gazebo_gps_plugin.h** shall be placed at **PX4-Autopilot/Tools/sitl_gazebo/include**

## Work_sim_spoof
This is old work and shouldn't be used.

## Thesis
This is for the thesis where the job is to detect spoofing

The ES-EKF.py has the **Error-state Extended Kalman Filter(ES-EKF)** and **Spoofing Detection** included.

### Simulation_GNSS_attacks
In this folder contains the 4 files. 2 of them is to spoof and 2 others is to send the position to the drone:

#### Spoofing attack simulatioin
* GNSS_spoofing.py --> If the GPS-plugin in Gazebo is used, should this run for being able to spoof the signal. It converts the GPS to UTM and back agian to GPS.
* OptiTrack_spoofing.py --> If the OptiTack is used in SITL or physical, should this run for being able to spoof the signal.

#### Simulate GNSS signal - OptiTrack
* relay.py --> This program should run if the physical OptiTrack is used.
* Relay_gazebo.py --> This prorgam should be used if the SITL for the OptiTrack is used.
