# Tracking Daisy

In this tutorial we use the [OptiTrack system installed] in our lab and the ROS
driver [mocap_optitrack] to get the very accurate pose of Daisy.


## Installation

Install [mocap_optitrack] (OptiTrack driver for ROS) on your notebook connected
to [Daisy's network].

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/mocap_optitrack.git
cd ~/catkin_ws
catkin_make
```


## Setup


### mocap_optitrack

Edit following settings in the source directory of [mocap_optitrack] downloaded
before.

File: `mocap_optitrack/config/mocap.yaml`

```yaml
#
# Definition of all trackable objects
# Identifier corresponds to Trackable ID set in Tracking Tools
#
rigid_bodies:
    '1':
        pose: daisy/pose
        pose2d: daisy/ground_pose
        child_frame_id: daisy/base_link
        parent_frame_id: world
optitrack_config:
        multicast_address: 224.0.0.1
```


## Usage

### OptiTrack

* Get the OptiTrack hardware key (Denise or other assistant).
* Start the PC and run `Motive`.
* Open the Project `daisy.ttf` (loaded by default).
* Check IP is set to [Daisy's network].

### ROS

* On your notebook start `mocap`:
  ```bash
  roslaunch mocap_optitrack mocap.launch
  ```
* By executing `rostopic list` you will find the topic for the pose of Daisy in
  the `world` frame.


## References

* [OptiTrack system installed]
* [mocap_optitrack]
* [OptiTrack Wiki](https://v20.wiki.optitrack.com)
* [Daisy's network]


[OptiTrack system installed]: ../../optitrack-and-ros/README.md
[mocap_optitrack]: http://wiki.ros.org/mocap_optitrack
[Daisy's network]: ../README.md

---
2017-12-07 | Denise Ratasich
