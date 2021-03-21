## Programming a Real Self-Driving Car
This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree. In this project, I completed ROS modules for updating waypoints, controllers for steering, brake, and speed controls, and for safely bringing the car to a stop when encountering a red light.

### Dependencies

* Follow these instructions to install ROS
  * [ROS Noetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu Ubuntu 20.04.2 LTS.
  * [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/macvincent/Integration.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator