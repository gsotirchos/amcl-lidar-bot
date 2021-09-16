# amcl-lidar-bot

## Overview

This is a package containing a 4-wheeled skid-steer drive robot equipped with a camera and a Hokuyo lidar in a Gazebo environment. A pre-generated costmap is provided to the `amcl` node along with the laser measurements and the robot localizes itself while it moves via move commands with the `move_base` or `teleop_switch_keyboard` nodes.

### License

The source code is released under an [MIT license](LICENSE).

**Author/Maintainer: George Sotirchos**

The amcl-lidar-bot package has been tested under [ROS](https://www.ros.org) Kinetic in a docker container on Ubuntu 20.04 (see [Running in Docker](#running-in-docker) section). This is experimental, personal project code, and possibly subject to frequent changes with any need for explanation disclaimed.

![Example image](media/recording.gif)

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, with ROS Kinetic on Ubuntu 16.04, clone the latest version from this repository into your catkin workspace and compile the package using

``` bash
mkdir -p /catkin_ws/src
cd catkin_ws/src
git clone https://github.com/7555G/amcl-lidar-bot
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

### Running in Docker

Install [Docker](https://docs.docker.com/get-docker/).

Spin up a container with GUI forwarding for X11 applications:

``` bash
docker run \
    -ti \
    --rm \
    --network=host \
    --env="DISPLAY" \
    --env QT_X11_NO_MITSHM=1 \
    --device=/dev/dri:/dev/dri \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    --name ros-container \
    mjenz/ros-kinetic-desktop-full \
    bash
```

This downloads the `mjenz/ros-kinetic-desktop-full` image from [mjenz](https://hub.docker.com/u/mjenz)'s Docker repository, indicates that it requires an interactive terminal (`-t`, `-i`), gives it a name (`--name`), removes it after you exit the container (`--rm`), sets the required environment variables (`--env`) and access to local resources (`--device`, `--volume`) to be able to launch graphical applications (Gazebo, RViz, rqt_graph, etc.), and runs a command (`bash`).

Now, continue with the instructions from the [Building](#building) section.

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

## Launch files

* **my_robot/launch/world.launch:** A Gazebo simulation is opened with 4-wheeled skid-steer drive robot equipped with a camera and a Hokuyo lidar spawned in an office environment, along with an RViz window showing the camera image and the lidar measurements on a pre-generated costmap.

* **ball_chaser/launch/amcl.launch:** Starts the `map_server`, `amcl`, and `move_base`/`teleop_switch_keyboard` nodes and loads their configurations.

     Arguments:

     - **`use_teleop`**: Whether to control the robot with a teleop node instead of the default move_base node.<br/>
        Default: `false` (Optional)

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/7555G/amcl-lidar-bot/issues).

ROS: http://www.ros.org<br/>
ROS Navigation Stack: https://github.com/ros-planning/navigation
