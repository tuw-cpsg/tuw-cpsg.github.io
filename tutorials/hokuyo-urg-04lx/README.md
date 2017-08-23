# Hokuyo URG-04LX

Laser rangefinder: 5V, 4m, 10Hz, compact.

## Connect the Hokuyo

The Hokuyo URG-04LX has a separate power supply via the pins, see the label on
the device. Connect the first 2 pins (5V and GND) to a 5V power supply
providing at least 500mA. For example, the Pioneer's user control board /
interface on top of the rover provides enough power via the USB sockets.

Connect and stream the range data via a USB-A to USB-A-mini cable.

* [Official Website](https://www.hokuyo-aut.jp/search/single.php?serial=165)
* [Specification](https://www.roscomponents.com/en/lidar-laser-scanner/83-urg-04lx-ug01.html)

## Install the ROS drivers

```bash
cd ~/catkin_ws/src/
git clone https://github.com/ros-drivers/driver_common.git
catkin_make -C ..
git clone https://github.com/ros-drivers/hokuyo_node.git
catkin_make -C ..
```

Compile the driver_common package first (see
also
[hokuyo_node install error](https://answers.ros.org/question/243232/how-to-install-hokuyo_nodeurg_node-on-kinetic/)).

## Usage

Check with `dmesg` or `ls /dev/ttyACM*` if and on which port the device is
connected, e.g., `/dev/ttyACM3`.

Check working communication with `rosrun hokuyo_node getID /dev/ttyACM3`.

```bash
roscore &
rosrun hokuyo_node hokuyo_node _port:=/dev/ttyACM3
```

The node will publish to topic `/scan` by default.

## Visualization

Start `rviz`, add LaserScan, change its topic to `/scan` and set `laser` as
fixed frame.

### Bugs/Problems

The new ROS driver for Hokuyo rangefinders [urg_node] does not work with this
sensor (not connected error). Check out [urg_node_04lx-github-issue] if they
fixed the issue. However it works with the depricated [hokuyo_node] (tested
with branch indigo-devel).

[urg_node]: http://wiki.ros.org/urg_node
[hokuyo_node]: http://wiki.ros.org/hokuyo_node
[urg_node_04lx-github-issue]: https://github.com/ros-drivers/urg_node/issues/23
