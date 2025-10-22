Overview
========

PX4 SITL setup using Gazebo and ROS2. Generally follows
<https://docs.px4.io/main/en/sim_gazebo_gz/>, but uses custom bringup scripts.

Usage
=====

Building
--------

- Compilation is performed with <https://github.com/asherikov/ccws>, although
  basic colcon build should work too.

- Build dependencies can be installed with `rosdep` except a few PX4 python
  dependencies that are installed during compilation with cmake.

Running
-------

- `px4sitl.sh`


Troubleshooting
===============

- `protobuf` like the following could be caused by multiple Gazebo versions
  installed in parallel: `[libprotobuf ERROR
  google/protobuf/descriptor_database.cc:121] File already exists in database:
  gz/msgs/pointcloud.proto`
