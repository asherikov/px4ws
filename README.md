Overview
========

<table>
  <tr>
    <td align="center">
        CI status
    </td>
    <td align="center">
        Debian package
    </td>
  </tr>
  <tr>
    <td align="center">
        <a href="https://github.com/asherikov/px4ws/actions/workflows/main.yaml">
        <img src="https://github.com/asherikov/px4ws/actions/workflows/main.yaml/badge.svg" alt="Build Status">
        </a>
    </td>
    <td align="center">
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/px4ws--reldebug--all/latest/a=amd64;d=ubuntu%252Fnoble;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/px4ws--reldebug--all/latest/a=amd64;d=ubuntu%252Fnoble;t=binary/?render=true&show_latest=true" alt="Latest version of 'px4ws' @ Cloudsmith">
        </a>
    </td>
  </tr>
</table>


Multi-drone PX4 SITL setup using Gazebo and ROS2. Generally follows
<https://docs.px4.io/main/en/sim_gazebo_gz/>, but uses custom bringup scripts
which serve as a showcase of <https://github.com/asherikov/cdinit>.

Workspace documentation is at <http://www.sherikov.net/px4ws/>.


Usage
=====

Building
--------

- Build `px4sitl` master package. Workspace is designed to be used with
  <https://github.com/asherikov/ccws>, but basic `colcon` build should work
  too.

- Dependencies can be installed with `rosdep` except a few PX4 python
  dependencies that are installed with cmake during build step.

Binary package
--------------

- Go to `cloudsmith` package repo, add new apt source, and run apt update.
- `sudo apt install px4ws--reldebug--all`
- `source /opt/px4ws/px4ws__reldebug__all/setup.bash`
- Launch as explained below.

Running
-------

- Start services:

```
cdinit.sh start px4sitl_ros@gui px4sitl_drone_x500@gui PX4SITL_WORLD_SDF_NAME=default
```

- Terminate by closing Gazebo gui, or with `cdinit.sh shutdown`. Since all
  processes are required, termination can also be performed by stopping a
  particular service, e.g., `cdinit.sh stop --force px4sitl_gz_clock`.

cdinit services
---------------

All components are started using `dinit` service files located in
`px4sitl/cdinit_services/`, see <https://github.com/asherikov/cdinit> for more
information.

### Example

- Start services as described above.
- List services `cdinit.sh list | sort` to get something like
```
# dummy "boot" service
[[+]     ] cdinit_main
# brings up a x500 drone
[[+]     ] px4sitl_drone_x500@gui
# brings up gazebo (with gui) and ROS bridge
[[+]     ] px4sitl_ros@gui
# logging services
[{+}     ] cdinit_log@px4sitl_gz_sim_gui
[{+}     ] cdinit_log@px4sitl_gz_sim_headless
[{+}     ] cdinit_log@px4sitl_gz_wait
[{+}     ] cdinit_log@px4sitl_px4_gui@.../px4sitl/config/x500.drone
[{+}     ] cdinit_log@px4sitl_px4_headless@.../px4sitl/config/x500.drone
[{+}     ] cdinit_log@px4sitl_ros_dds_agent
[{+}     ] cdinit_log@px4sitl_ros_gz_clock
[{+}     ] cdinit_sessionsync
# brings up Gazebo
[{+}     ] px4sitl_gz_sim@gui
# Gazebo gui
[{+}     ] px4sitl_gz_sim_gui
# Gazebo simulation core
[{+}     ] px4sitl_gz_sim_headless
# Helper service which waits for Gazebo to start
[{+}     ] px4sitl_gz_wait
# QGroundControl
[{+}     ] px4sitl_px4_gui@.../px4sitl/config/x500.drone
# PX4 instance with corresponding configuration file
[{+}     ] px4sitl_px4_headless@.../px4sitl/config/x500.drone
# Micro-XRCE-DDS-Agent bridging PX4 and ros2
[{+}     ] px4sitl_ros_dds_agent
# Gazebo -> ROS2 simulation clock bridge
[{+}     ] px4sitl_ros_gz_clock
```
- Logs are located in cdinit session root directory printed on startup, it
  depends on environment variables, e.g., `ROS_LOG_DIR`.

### Service dependency graph

<img src="https://raw.githubusercontent.com/asherikov/px4ws/refs/heads/main/doc/service_graph.svg" alt="service graph" />


Troubleshooting
===============

- `protobuf` errors like the following could be caused by multiple Gazebo
  versions installed in parallel: `[libprotobuf ERROR
  google/protobuf/descriptor_database.cc:121] File already exists in database:
  gz/msgs/pointcloud.proto`
- PX4 fails during automatic takeoff with "Disarmed by auto preflight
  disarming": make sure that unused parameters in vehicle command message are
  set to NaN.

TODO
====

- Is installation of `px4/build/` intentional?
- `CMakeLists.txt`, `package.xml` should not be installed.
