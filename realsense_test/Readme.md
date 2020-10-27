# Realsense Test
This repository aims at defining realsense stress tests. On every reboot of the system a `realsense.service` starts which launches both cameras connected, and monitors particular topic being published from them. If the topics are not published with a timeout threshold, it is logged as a failure. If more than `10` messages are received from both cameras, within the timeout threshold, we call it a success and log it into the file. The program is then put to sleep for some time and then the system is rebooted. This process happens for a certain number of iterations defined by the `itr` argument in the launch file defined later.

## Usage
The following assumes this is being run on a deployment ready IPC, with at least the following requirement:
- `/etc/ros/setup.bash` file sources `/opt/por` and defines the `HOME` environment variable
- It is allowed to install files at `/opt/por`
- Super user privileges are available
- An `autoboot` user exists with `sudo` privileges
- `librealsense v2.20.0` is installed from upstream (Intel)

#### Reboot w/o sudo for autoboot user
```
sudo su
echo "autoboot ALL=NOPASSWD: /sbin/reboot">>etc/sudoers
```
#### Create workspace
```
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone -b am_realsense_test https://github.com/abhijitmajumdar/realsense.git
```
#### Installation
```
cd ~/realsense_ws
sudo su
source /opt/ros/kinetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release install -DCMAKE_INSTALL_PREFIX=/opt/por
exit
```
#### Schedule a startup service
```
cd ~/realsense_ws
source devel/setup.bash
rosrun robot_upstart install realsense_test/launch/realsense_reboot_test.launch --provider systemd --user autoboot --setup /etc/ros/setup.bash
sudo systemctl daemon-reload && sudo systemctl start realsense
```
#### Enable service to start on system boot automatically
Enabling:
```
sudo systemctl enable realsense.service
```
Disabling:
```
sudo systemctl disable realsense.service
```
#### Check service status
Make sure the service has not crashed and is still running
```
systemctl status realsense.service
```

## Emergency stop
If at any time, the use wants to interrupt the reboot test process, please do the following:
```
sudo systemctl disable realsense.service
```
This will disable the service from startup. Once the system reboots again, the system should be usable in a normal state


## Modifications
The arguments for the launch file `realsense_reboot_test.launch` provides control on the reboot test parameters and could be modified accordingly
- `itr`: The maximum number of times to reboot the system to perform this test
- `logfile`: The filename for logging the results on each boot which is placed in the `\home\autoboot` directory
- `cam1_serial`: The serial number for camera 1
- `cam2_serial`: The serial number for camera 2
- `sleep`: The time to sleep in minutes before rebooting the system after assessing a success/failure to repeat the test
- `timeout`: The timeout in minutes to monitor the topics after which the test is deemed a failure if there are no published topics or the camera(s) fail
- `cam1_topic`: The topic on camera 1 to monitor
- `cam2_topic`: The topic on camera 2 to monitor
