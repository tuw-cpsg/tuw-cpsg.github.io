# OptiTrack/Motion + ROS

In order to receive data in ROS from an OptiTrack system running the Motive software, there are two ways that were investigated:
- OptiTrack (NatNet) streaming engine with `mocap_optitrack` ROS package
- VRPN streaming engine with `vrpn-client-ros` ROS package

The following describes the ways to install and use the two possible packages.

## Basic Setup

The basic setup consists of the OptiTrack motion capturing system, a PC running the OptiTrack's Motive software and one PC running ROS.
The PC with the Motive Software needs to be connected to the OptiTrack cameras via ethernet to the OptiTrack switch.
In an optimal fashion, the PC is connected to a second network that is used to stream the information.
The other PC, running ROS, is then connected to the second network that is used to stream the information.

![Basic setup using the OptiTrack system, a PC running Motive and another PC running ROS.][setup]

To create a rigid body for streaming, select the markers of the object (at least three) and right-click in Motive and select `Rigid Body -> Create From Selected Markers`.

![Create a rigid body.][create_rigid_body]

Then you see a new rigid body in the `Assets` in the project pane.
To open the project pane click on `View -> Project`.

![Project pane showing all tracked rigid bodies.][project_object]

Note that only those objects will be streamed where the checkbox is checked in the project pane.

## OptiTrack/NatNet Streaming Engine ([mocap_optitrack])

[NatNet] is the OptiTrack streaming engine used to stream data over the network.
It has a powerful integration in the OptiTrack's Motive software.
The available `mocap_optitrack` ROS package can receive and publish the streamed information as topics and TFs.

### Installation

```bash
cd $MR_DIR/src
git clone git@github.com:ros-drivers/mocap_optitrack.git
cd $MR_DIR ; catkin_make
```

### Configuring Motive

Open the [Data Streaming Pane] in Motive by clicking on `View->Data Streaming`.
The first checkbox `Broadcast Frame Data` in the `OptiTrack Streaming Engine` group activates the OptiTrack streaming engine and with the following settings the streamed data is configured.

To stream the data to the ROS PC, the proper `Network Interface` has to be selected in `Local Interface` by selecting the PC's IP address of the network to which the ROS PC is connected to.
In order to activate streaming of rigid bodies, set the option `Stream Rigid Bodies` to `True`.

Finally set `Type` in `Advanced Network Settings` to `Multicast`, specify (command and data) ports if necessary and set the `Multicast Interface` (default: 239.255.42.99).

![OptiTrack streaming engine settings part 1 of 2.][natnet1] ![OptiTrack streaming engine settings part 2 of 2.][natnet2]

### Configuring [mocap_optitrack]

The [mocap_optitrack] packages comes with an ready-to-use launch file.
This launch file loads a configuration (default: config/mocap.yaml) file, wich sets all the necessary information to communicate with Motive.
A basic configuration looks as follows:

```
rigid_bodies:
    '1':
        pose: Robot_1/pose
        pose2d: Robot_1/ground_pose
        child_frame_id: Robot_1/base_link
        parent_frame_id: world
optitrack_config:
        multicast_address: 224.0.0.1
```

The section under `rigid_bodies` defines all the rigid bodies that will be tracked.
Their ID needs to match the `User ID` defined in the project pane in Motive.
`pose` and `pose2d` define the topics to which the streamed data will be published.
`child_frame_id` and `parent_frame_id` define the TF that will be published.

In the `optitrack_config` section all necessary information to communicate with Motive will be set.
The `multicast_address` defined here needs to match the `Multicast Interface` defined in Motive.

Note that the default value for the `multicast_address` in [mocap_optitrack] is `224.0.0.1` whereas the default value in Motive for `Multicast Interface` is `239.255.42.99`!

### Results

RViz visualizing the TF and the pose streamed from the optitrack system.

```bash
roslaunch mocap_optitrack mocap.launch & rosrun rviz rviz
```

![RViz visualizing the streamed data using the OptiTrack streaming engine.][mocap_rviz]

### Bugs/Problems

When multiple objects are streamed from motive, [mocap_optitrack] crashes.
The [bug](https://github.com/ros-drivers/mocap_optitrack/issues/23) is already filed, and a [fix](https://github.com/ros-drivers/mocap_optitrack/pull/24) has already been published.
However, the fix is not merged into the master because it is not tested so far.

## VRPN Streaming Engine ([vrpn_client_ros])

[VRPN] is the virtual reality peripheral network.
The streaming engine is implemented in the OptiTrack's Motive software to stream VRPN data over the network.
However, it is only possible to stream 6 DOF rigid body data via VRPN.

### Installation

```bash
sudo apt-get install ros-kinetic-vrpn-client-ros
```

### Configuring Motive

To activate the [VRPN] streaming engine, activate the checkbox `Broadcast Frame Data` in the `VRPN Streaming Engine` group.
Set the `VRPN Broadcast Port` if necessary (default: 3883).

![vrpn_streaming]

In the project pane, the `Name` for the object to be tracked must not have any whitespaces.
To rename the object click with the right mous button on the object and select `Rename Asset`.

### Configuring [vrpn_client_ros]

A basic launch file to start the VRPN client for ROS:

```xml
<launch>
  <arg name="server" default="128.130.39.61"/>
  <node pkg="vrpn_client_ros" type="vrpn_client_node" name="vrpn_client_node" output="screen">
    <rosparam subst_value="true">
      server: $(arg server)
      port: 3883
      frame_id: world
      broadcast_tf: true
      # Must either specify refresh frequency > 0.0, or a list of trackers to create
      refresh_tracker_frequency: 1.0
      #trackers:
      #- FirstTracker
      #- SecondTracker
    </rosparam>
  </node>
</launch>
```

The `server` argument specifies the server's IP address.
`refresh_tracker_frequency` specifies the frequency for how often the client looks for new objects.
This means that new objects that motive will stream in future will be recognized automatically.
If set to `0` (default), the trackers have to be specified manually.
To publish a TF, `broadcast_tf` needs to be set to `true`.
Further configuration settings can be found on the [vrpn_client_ros] wiki page.

### Results

RViz visualizing the TF and the pose streamed from the optitrack system.

```bash
roslaunch sandbox vrpn.launch server:=128.130.39.61 & rosrun rviz rviz
```

![RViz visualizing the streamed data using the VRPN streaming engine.][vrpn_rviz]

### Bugs/Problems

It seems that there is an coordiante system mismatch. The rviz results in [mocap_optitrack] and in [vrpn_client_ros] show the same position.
However, in [vrpn_client_ros] the `y` and `z` coordinates seem to be switched which points to the fact that there is a right- vs. left-handed cooridante system problem.

[NatNet]: http://www.optitrack.com/products/natnet-sdk/
[VRPN]: https://github.com/vrpn/vrpn/wiki
[mocap_optitrack]: http://wiki.ros.org/mocap_optitrack
[vrpn_client_ros]: http://wiki.ros.org/vrpn_client_ros
[setup]: ./setup.png "Basic OptiTrack + ROS Setup"
[Data Streaming Pane]: http://wiki.optitrack.com/index.php?title=Data_Streaming_Pane

[create_rigid_body]: ./create_rigid_body.png "Create a rigid body from selected markers."
[data_streaming]: ./data_streaming.png "Open the data streaming pane."
[mocap_rviz]: ./mocap_rviz.png "RViz visualizing the TF and the pose received from Motive via OptiTrack streaming engine."
[natnet1]: ./natnet1.png "OptiTrack streaming engine settings part 1 of 2."
[natnet2]: ./natnet2.png "OptiTrack streaming engine settings part 2 of 2."
[project]: ./project.png "Open the project pane."
[project_object]: ./project_object.png "Object settings in the project pane."
[vrpn_rviz]: ./vrpn_rviz.png "RViz visualizing the TF and the pose received from Motive via VRPN streaming engine."
[vrpn_streaming]: ./vrpn.png "VRPN streaming engine settings."
