# periodic_slam

## Overview

This package performs state estimation periodically

Dataset: 

## Citation
If you use this work, please be sure to cite [Kumar, H.; Payne, J. J.; Travers, M.; Johnson, A. M.; and Choset, H. Periodic SLAM: Using Cyclic Constraints to Improve the Performance of Visual-Inertial SLAM on Legged Robots. In ICRA Workshop on Visual-Inertial Navigation Systems, May 2021.](https://udel.edu/~ghuang/icra21-vins-workshop/papers/04-Kumar_PeriodicSLAM.pdf)
## Preliminaries

This package uses: 
* the feature tracker and image processor nodelet included with [msckf_vio] (https://github.com/KumarRobotics/msckf_vio).

## Installation

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)
- [gtsam](https://github.com/borglab/gtsam) (backend graph optimization) When building gtsam from source, use the following cmake flags: -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WRAP=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_TYPEDEF_POINTS_TO_VECTORS=ON 
- [OpenCV] 

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/kumarhans/periodic_slam.git
	cd ../
	catkin_make


## Usage

Run the main visual odometry node with

	roslaunch periodic_slam cameraAndFactor.launch
	

## NEED TO FINISH WRITING REST OF README

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")
