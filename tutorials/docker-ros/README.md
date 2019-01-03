# ROS with Docker


## Installation

* Install docker (see installation instructions for your platform on [docker
  docs - About Docker CE] and [post-installation
  steps](https://docs.docker.com/install/linux/linux-postinstall/)).

* If you want to use ROS GUI tools build a custom docker image (no official
  desktop-full images?):
  ```bash
  $ cat Dockerfile
  FROM ros:kinetic
  RUN apt-get update && apt-get install -y \
      ros-kinetic-desktop-full
  $ docker build -t ros:kinetic-desktop-full .
  ```

  If you only use the CLI, simply pull the ROS docker image:
  ```bash
  $ docker pull ros:kinetic
  ```
  I pulled `ros:kinetic` to have the ros package `p2os_msgs` available as
  binary which was not (yet) in ROS melodic repository on 2018-09-12. You may
  try the latest image.

* Run ROS container:
  ```bash
  $ docker run \
      -it \
      -v ~/catkin_ws:/root/catkin_ws \
      -v ~/.ssh:/root/.ssh \
      --network host \
      --env ROS_MASTER_URI=http://<your hostname>:11311 \
      --name ros
      ros:kinetic[-desktop-full]
  ```

  This command starts the ROS docker image `ros` in an interactive terminal
  mode (`-i -t`) and mounts the host catkin workspace `~/catkin_ws` to the home
  directory of the running container. Note, mounting does not resolve symlinks!

  The container uses the host's network with *no isolation (!)* (as if the
  container processes would run on the host) using the host's ssh
  configuration, `/etc/hosts` and hostname. The environment variable of the
  master URI is set, however, you might have a ros-entrypoint-script sourcing
  the workspaces anyway where you can add the export of the variable.

  It is convinient to assign a name to the container:
  * to start another shell in the same container (this is not the typical
    Docker way to do it -- one process per container).
    ```bash
    $ docker exec -it ros bash
    ```
  * to stop the container.
    ```bash
    $ docker stop ros
    ```
  * to restart the container.
    ```bash
    $ docker start ros
    ```


## Usage Examples

### Useful Docker commands
```bash
$ docker ps
$ docker ps -a
$ docker image ls
$ docker info
$ docker start <name>
$ docker exec -it <name> bash
$ docker stop <name>
$ docker container rm <name>
```

### Inspect ROS nodes

On the host do:

* Run the ROS docker container created before:
  ```bash
  $ docker start ros
  $ docker exec -it ros bash
  ```

  Start some nodes in the container, e.g.:
  ```bash
  $ source /opt/ros/kinetic/setup.sh
  $ roscore &
    :
  $ rostopic pub -r 1 /test std_msgs/String "test"
  ```

* Run a docker container executing `rqt_graph`:
  ```bash
  $ docker run \
      --net=host \
      --user $(id -u) \
      --env DISPLAY=$DISPLAY \
      -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --name rqt_graph \
      ros:kinetic-desktop-full \
      rqt_graph
  ```

  Use the host's network as before to be part of the ROS network. The GUI shall
  be executed by the current user -- not root (see also [ROS Docker and GUI]).
  Note that `--user=$USER` did not work for me. The name is unknown but the UID
  works. The display X server is shared.

  Give the container a name to be able to restart it later, select the
  desktop-full image and execute `rqt_graph`.

You may create an alias for convinience (in `.bashrc` or similar):
```bash
alias docker_gui="docker run --net=host --user $(id -u) --env DISPLAY=$DISPLAY -v '/tmp/.X11-unix:/tmp/.X11-unix:rw'"
```
So you can later start a ROS GUI tool by:
```bash
$ docker_gui --name rqt_graph ros:kinetic-desktop-full rqt_graph
```
You can then (re-)start `rqt_graph` by:
```bash
$ docker start rqt_graph
```

Some GUI apps still make troubles (e.g., rviz). You may try [x11docker].

### Controlling Daisy

On the host do:

* Setup the network with Daisy. You will need to connect to [Daisy's network]
  install `openssh-server` and exchange SSH keys (see [network setup]).

* Create a folder for your [ROS workspace] and download the [general-ros-modules
  repository] including the tele-operation package:
  ```bash
  $ mkdir -p ~/catkin_ws/src
  $ cd ~/catkin_ws/src
  $ git clone https://github.com/tuw-cpsg/general-ros-modules.git
  ```

* Run ROS in docker container created before.
  ```bash
  $ docker start ros
  $ docker exec -it ros bash
  ```

In the container do:

* Install `p2os_msgs` (replace `kinetic` with your distro) which is needed to
  communicate with the robot driver (e.g., motor control). `bash-completion`
  is just for convinience.
  ```bash
  $ apt update
  $ apt install bash-completion
  $ apt install ros-kinetic-p2os-msgs
  ```

* Initialize the [ROS workspace] (build and source):
  ```bash
  $ source /opt/ros/kinetic/setup.sh
  $ cd ~/catkin_ws
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
* [ROS Docker and GUI] -- outdated but still useful instructions to run ROS with Docker
* [ROS Docker images] -- ROS images and examples
* [ROS workspace] -- how to create a ROS workspace
* [x11docker] -- the simple way to run GUI apps in docker
* [Daisy Demos with Docker] -- images to control Daisy with ROS on docker


[Docker Documentation]: https://docs.docker.com/
[docker docs - About Docker CE]: https://docs.docker.com/install/
[ROS Docker and GUI]: http://wiki.ros.org/docker/Tutorials/GUI
[ROS Docker images]: https://hub.docker.com/_/ros/
[ROS workspace]: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
[Daisy's network]: ../README.md
[network setup]: ../../dagobert-network-setup.md
[general-ros-modules repository]: https://github.com/tuw-cpsg/general-ros-modules
[x11docker]: https://github.com/mviereck/x11docker
[Daisy Demos with Docker]: https://github.com/dratasich/docker

---
2018-01-03 | Denise Ratasich
