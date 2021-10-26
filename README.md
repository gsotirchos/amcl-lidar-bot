# amcl-lidar-bot

## Overview

This is a meta-package containing two packages:
* **my_robot**: A 4-wheeled skid-steer drive robot equipped with a camera and a Hokuyo lidar in a Gazebo environment. A pre-generated costmap is provided to the `amcl` node along with the laser measurements. The robot localizes itself while controlled manually via move commands with the `move_base` node or via the keyboard with the `teleop_switch_keyboard` nodes. Optionally, the `surveyor` package can be used for simple autonomous motion.
* **surveyor**: Processes the laser measurements and drives the robot accordingly. The direction of the farthest measured distance is determined and the robot is driven to move towards this direction.

### License

The source code is released under an [MIT license](LICENSE).

**Author/Maintainer: George Sotirchos**

The amcl-lidar-bot package has been tested under [ROS](https://www.ros.org) Noetic on Ubuntu 20.04. This is experimental, personal project code, and possibly subject to frequent changes with any need for explanation disclaimed.

![Example image](media/recording.gif)

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, with ROS Kinetic on Ubuntu 20.04, clone the latest version from this repository into your catkin workspace and compile the package using

``` bash
mkdir -p /catkin_ws/src
cd catkin_ws/src
git clone https://github.com/7555G/amcl-lidar-bot
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```
## Usage

1. Start the Gazebo environment containing the robot:

    ``` bash
    roslaunch my_robot world.launch
    ```

2. Start the `map_server`, `amcl`, and `move_base` nodes:

    ``` bash
    roslaunch my_robot amcl.launch
    ```

    Or to use the `teleop_switch_keyboard` node for robot control instead of the `move_base` node:

    ``` bash
    roslaunch my_robot amcl.launch use_teleop:=true
    ```

3. Optionally, start the `drive_bot` and `process_laser` nodes to enable the robot to survey the area by itself.

    ``` bash
    roslaunch surveyor surveyor.launch
    ```

## Launch files

* **my_robot/launch/world.launch:** A Gazebo simulation is opened with 4-wheeled skid-steer drive robot equipped with a camera and a Hokuyo lidar spawned in an office environment, along with an RViz window showing the camera image and the lidar measurements on a pre-generated costmap.

* **my_robot/launch/amcl.launch:** Starts the `map_server`, `amcl`, and `move_base`/`teleop_switch_keyboard` nodes and loads their configurations.

     Arguments:

     - **`use_teleop`**: Whether to control the robot with a teleop node instead of the default move_base node.<br/>
        Default: `false` (Optional)

* **surveyor/launch/surveyor.launch**: Starts the `drive_bot` and `process_laser` nodes which analyze the laser measurements and drive the robot autonomously.

## Nodes

### drive_bot

Provides a service for publishing drive commands to the robots actuators.

#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    The requested `linear_x` and `angular_z` velocities for the robot wheel joints.

#### Services

* **`~command_robot`** ([surveyor/DriveToTarget](surveyor/srv/DriveToTarget.srv))

    Publishes the requested the requested `linear_x` and `angular_z` velocities to `/cmd_vel` topic.

    ```
    rosservice call /surveyor/command_robot "linear_x: 0.0
    angular_z: 0.0"  # with the newline
    ```

### process_laser

Processes the laser scan published at `/scan` and requests the appropriate drive commands via the `~command_robot` service.

#### Subscribed Topics

* **`/scan`** ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html))

    The robot-mount Lidar measurements.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/7555G/amcl-lidar-bot/issues).

ROS: http://www.ros.org<br/>
ROS Navigation Stack: https://github.com/ros-planning/navigation
