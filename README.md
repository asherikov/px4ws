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


Packages
--------
```
PX4-Autopilot/PX4-Autopilot                         main             https://github.com/asherikov/PX4-Autopilot.git
cdinit                                              master           https://github.com/asherikov/cdinit.git
dds_agent/Micro-XRCE-DDS-Agent/Micro-XRCE-DDS-Agent v2.4.3           https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
dds_agent/spdlog                                    v1.9.2           https://github.com/gabime/spdlog.git
px4_msgs                                            as_disable_tests https://github.com/asherikov/px4_msgs.git
```
`PX4-Autopilot` and `Micro-XRCE-DDS-Agent` are build using proxy packages in
order to apply a few `cmake` tweaks


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

- Run `px4sitl.sh -w walls -d 4001 "0,1.0,0,0,0,0" -d 4004 "0,-5.0,0,0,0,0"`
  where `-w` specifies world ('default' if not set), `-d` specifies a drone
  model id and its location (x,y,z,r,p,y). Numerical ids can be found in a
  table at <https://docs.px4.io/main/en/sim_gazebo_gz/>.
- `px4sitl.sh` also supports headless mode, which is enabled by passing `-H`
  flag.
- Terminate by closing Gazebo gui, or with `cdinit.sh shutdown`.

cdinit services
---------------

All components are started using `dinit` service files located in
`px4sitl/cdinit_services/`, see <https://github.com/davmac314/dinit>,
<https://github.com/asherikov/cdinit> provides a cmake wrapper for `dinit` and
helper scripts. Shell scripts are primarily used to setup environment and
working directories.

### Example

- Start `px4sitl.sh -w walls -d 4001 "0,1.0,0,0,0,0" -d 4004 "0,-5.0,0,0,0,0"`
- List services `cdinit.sh list | sort` to get something like
```
# dummy "boot" service
[[+]     ] cdinit_main
# Gazebo gui
[[+]     ] px4sitl_gz_gui (pid: XXX)
# PX4 instances, number after @ symbol is a corresponding system id
[[+]     ] px4sitl_px4@0 (pid: XXX)
[[+]     ] px4sitl_px4@1 (pid: XXX)
# logging services, cdinit_log helper adds timestamps
[{+}     ] cdinit_log@px4sitl_dds_agent (pid: XXX)
[{+}     ] cdinit_log@px4sitl_gz_clock (pid: XXX)
[{+}     ] cdinit_log@px4sitl_gz_gui (pid: XXX)
[{+}     ] cdinit_log@px4sitl_gz_headless (pid: XXX)
[{+}     ] cdinit_log@px4sitl_gz_wait (pid: XXX)
[{+}     ] cdinit_log@px4sitl_px4@0 (pid: XXX)
[{+}     ] cdinit_log@px4sitl_px4@1 (pid: XXX)
# Micro-XRCE-DDS-Agent bridging PX4 and ros2
[{+}     ] px4sitl_dds_agent (pid: XXX)
# Gazebo -> ROS2 simulation clock bridge
[{+}     ] px4sitl_gz_clock (pid: XXX)
# Gazebo simulation core
[{+}     ] px4sitl_gz_headless (pid: XXX)
# Helper service which waits for Gazebo to start
[{+}     ] px4sitl_gz_wait
# Dummy "master" service
[{+}     ] px4sitl_ros
```
- Location of the log files depends on environment variables, e.g.,
  `ROS_LOG_DIR` and is printed on startup.
- Terminate using `cdinit.sh shutdown`. Since all processes are required,
  termination can also be performed by stopping a particular service, e.g.,
  `cdinit.sh stop --force px4sitl_gz_clock`.


Troubleshooting
===============

- `protobuf` errors like the following could be caused by multiple Gazebo
  versions installed in parallel: `[libprotobuf ERROR
  google/protobuf/descriptor_database.cc:121] File already exists in database:
  gz/msgs/pointcloud.proto`

TODO
====

- `FindOpticalFlow.cmake` is installed in a wrong place.
- Is installation of `px4/build/` intentional?
- `CMakeLists.txt`, `package.xml` should not be installed.
- Is `spdlog` needed?
