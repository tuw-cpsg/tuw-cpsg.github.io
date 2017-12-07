Please do not change the basic networking settings (several students and
employees use the rovers). However, you can propose and implement better
solutions in consultation with Denise.


# Connect to `dagobert`

`dagobert` runs Ubuntu with ROS and provides an access point when
started. Denise manages the accounts and connection details (SSID, fixed IP,
personal username and password) -> contact for account creation.

Connect to `dagobert`'s SSID and ssh to the rover.


# Setup ROS network

It is more convenient to run your developed nodes on your own notebook and just
start the Pioneer driver on the rover (communicate with the rover directly via
ROS). This way you don't have to develop on the rover directly, i.e., you don't
need an internet connection, you don't have to copy/pull code, the rover
remains "clean", etc.

For that to work you need to be able to `ssh dagobert` and `ssh your-notebook`
vice versa (you will need `openssh-server` installed on your notebook).

* Select a static IP for your notebook.

* Add `dagobert` to your `/etc/hosts` file and add `your-notebook` to the
  `/etc/hosts` of the rover.

* Add `dagobert` and your username to your ssh config, e.g. (you can choose a
  specific ssh-key-pair by setting `IdentityFile .ssh/id_rsa` additionally):

  ```bash
  Host your-notebook
    HostName your-notebook
    User your-username-on-your-notebook
  ```

* Exchange ssh-keys such that you can ssh without
  password-prompt. Unfortunately, ROS connections can only be established with
  RSA keys ([known_hosts
  error](http://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/)).

  * If you already connected to the rover, by default the ECDSA key is added to
    the `known_hosts` in `~/.ssh`. You have to remove these keys with
    `ssh-keygen -R "dagobert"` or `ssh-keygen -R "dagobert-ip-address"`.

  * Then copy your notebook's public key **using RSA**: `ssh-copy-id -i
    /path/to/id_rsa.pub -oHostKeyAlgorithms='ssh-rsa' dagobert`. You should be
    asked to verify the RSA fingerprint. If there occurs another warning you
    may have missed to delete a key in `known_hosts`. You may have to ssh first
    (with rsa) and then copy the id.

  * The public key of your notebook should now be in the `.ssh/authorized_keys`
    file of your account on `dagobert`.

  * Perform the same for `dagobert` vice versa, i.e., call `ssh-copy-id` to
    `your-notebook`.

See also [ROS on multiple
machines](http://wiki.ros.org/ROS/Tutorials/MultipleMachines) and [ROS network
setup](http://wiki.ros.org/ROS/NetworkSetup).


# Test ROS network

The `pioneer_teleop` package in the [`general-ros-modules`
repository](https://github.com/tuw-cpsg/general-ros-modules) includes a launch
file to control the robot via your notebook (keyboard). It starts the
[p2os](http://wiki.ros.org/p2os-purdue) driver on the rover which controls the
motors given velocity commands (ROS topic `cmd_vel`).

Clone the repository in your catkin workspace (typically, `~/catkin_ws/src/`)
and build it. Connect to the rover's WLAN and execute the launch file on your
notebook (this will run the ROS master, teleoperation and additional nodes on
your notebook and only the driver on the robot):

```bash
$ roslaunch pioneer_teleop drive.launch \
  notebook:=your-notebook robot:=dagobert \
  robot-distro:=hydro robot-port:=/dev/ttyS0
```

For the tele-operation to work, the notebook has to know the messages from the
rover, i.e., install `ros-<distro>-p2os-msgs`.

On 17.04 the following error occured: pycrypto not installed. This error
vanished when I installed `python-pip`.
