# Controlling Daisy with ROS running in a Docker Container
(the simple way)

## Installation

* Setup the network with Daisy. You will need to connect to [Daisy's network]
  install `openssh-server` and exchange SSH keys (see [network setup]).

* Install docker (see installation instructions for your platform on [docker
  docs - About Docker CE] and [post-installation
  steps](https://docs.docker.com/install/linux/linux-postinstall/)).

* Pull the ROS docker image (see also [ROS + Docker Tutorials]):
  ```bash
  $ docker pull ros:kinetic
  ```
  I pulled `ros:kinetic` to have the ros package `p2os_msgs` available as
  binary (which was not in the sources on 2018-09-12). You may try with the
  latest image.

* Create a folder for your [ROS workspace] and download the [general-ros-modules
  repository] including the tele-operation package:
  ```bash
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/tuw-cpsg/general-ros-modules.git
  ```

* Run ROS container:
  ```bash
  $ docker run \
      -it \
      -v ~/catkin_ws:/root/catkin_ws \
      -v ~/.ssh:/root/.ssh \
      --network host \
      --env ROS_MASTER_URI=http://<your hostname>:11311 \
      --name ros
      ros:kinetic
  ```

  This command starts the ROS docker image `ros` in an interactive terminal
  mode (`-i -t`) and mounts the host catkin workspace `~/catkin_ws` to the home
  directory of the running container. Note, mounting does not resolve symlinks!

  The container uses the host's network with *no isolation (!)* (as if the
  container processes would run on the host) using the host's ssh
  configuration, `/etc/hosts` and hostname. The environment variable of the
  master URI is set.

  It is convinient to assign a name to the container:
  * to start another shell in the same container.
      docker exec -it ros bash
  * to stop the container.
      docker stop ros
  * to restart the container.
      docker restart ros

The same container can be used to start more shells via `docker exec -it
<container-id or -name> bash` (call `docker ps` for the container ID).

On the guest do:

* Install `p2os_msgs` (replace `kinetic` with your distro) which is needed to
  communicate with the robot driver (e.g., motor control). `bash-completion`
  is just for convinience.
  ```bash
  $ apt update
  $ apt install bash-completion
  $ apt install ros-kinetic-p2os-msgs
  ```


## Usage

On the host do:
```bash
$ docker restart ros
$ docker exec -it ros bash
```

Following commands are executed in the container.
On the guest do:

* Initialize the [ROS workspace] (build and source):
  ```bash
  $ source /opt/ros/kinetic/setup.sh
  $ cd /root/catkin_ws
  $ catkin_make
  $ source devel/setup.sh
  ```

* Launch the teleop node to control Daisy:
  ```bash
  $ roslaunch pioneer_teleop drive.launch \
      notebook:=<your hostname> \
      robot:=daisy robot-distro:=indigo robot-port:=/dev/ttyTHS1
  ```


## References

* [Docker Documentation]
* [docker docs - About Docker CE]
* [ROS Docker docs]
* [ROS + Docker Tutorials]
* [ROS Docker images]
* [ROS workspace]


[Docker Documentation]: https://docs.docker.com/
[docker docs - About Docker CE]: https://docs.docker.com/install/
[ROS Docker docs]: https://docs.docker.com/samples/library/ros/
[ROS + Docker Tutorials]: http://wiki.ros.org/action/fullsearch/docker/Tutorials/
[ROS Docker images]: https://hub.docker.com/_/ros/
[ROS workspace]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[Daisy's network]: ../README.md
[network setup]: ../../dagobert-network-setup.md
[general-ros-modules repository]: https://github.com/tuw-cpsg/general-ros-modules

---
2018-09-12 | Denise Ratasich
