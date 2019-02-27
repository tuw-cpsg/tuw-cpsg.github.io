# Time-of-Flight Camera


## Raspberry Pi `tof-rpi`

### Network Setup

Connects to some WiFi networks automatically (see Ubuntu MATE connection manager).
For instance, you could also [setup a hotspot] for `tof-rpi`.

An SSH server is installed. So you can `ssh <username>@<hostname>` to the RPi.
Use `ssh -A <username>@<hostname>` to forward identity, e.g., to work in a git repo with your credentials.

### ROS Setup

- Runs Ubuntu MATE 16.04 (Xenial).
- [Installed ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu).
- Installed `openssh-server` for ROS communication.
- Connect to [Daisy's network] (via network settings in the desktop environment).
- [SSH configuration](../dagobert-network-setup.md):
  - Put hosts into `/etc/hosts` to be able to communicate with daisy and my notebook.
  - [Create SSH key](https://help.github.com/en/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).
  - SSH configuration for all devices in the ROS network according to following template:
    ```bash
    # ROS network device
    Host <hostname>
        HostName <hostname>
        User <username>
        PreferredAuthentications publickey,password
        IdentityFile ~/.ssh/id_rsa
    ```
    ROS connects `tof-rpi` to `hostname` via `<username>@<hostname>` using `<rsakey>`.
  - Copy IDs (of nodes that will communicate to each other, e.g., only notebook and `tof-rpi`).
  - Create the executable environment loader script `/opt/ros/kinetic/env_pi.bash`:
    ```bash
    #!/usr/bin/env bash
    source /home/<username>/catkin_ws/devel/setup.bash
    exec "$@"
    ```


## Applications using ToF Camera

* The [monitor](https://github.com/dratasich/shsa-prolog)
  of the [self-healing package](https://github.com/dratasich/shsa_ros)
  is demonstrated with the ToF camera mounted to [Daisy]
  in the [SASO 2019 paper](https://github.com/tuw-cpsg/paper-shsa-monitor-experiments).

## Repositories using ToF Camera

* [Software interface](https://github.com/tuw-cpsg/tof-sw-interface) for the camera.
* [ROS driver](https://github.com/tuw-cpsg/tof-ros-interface) publishing point cloud of the depth to ROS.


[Daisy]: ../daisy/README.md
[setup a hotspot]: access-point/README.md

---
2019-02-27 | Denise Ratasich
