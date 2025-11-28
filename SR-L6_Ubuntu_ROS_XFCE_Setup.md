# Full Installation Guide: Ubuntu Server + XFCE + ROS + SR-L6 Robot Arm

## 1. Download Ubuntu Server & Create Bootable USB

-   Download Ubuntu Server 20.04 LTS.
-   Use Rufus (Windows) or this command (Linux):

``` bash
sudo dd if=ubuntu-server.iso of=/dev/sdX status=progress
```

## 2. Install Ubuntu Server

Follow installer: - Language: English\
- Keyboard: English (India)\
- Network: Connect WiFi/Ethernet\
- Install OpenSSH → YES\
- Reboot after installation

## 3. Update System

``` bash
sudo apt update
sudo apt upgrade -y
```

## 4. Install XFCE GUI

``` bash
sudo apt install xfce4 xfce4-goodies -y
sudo apt install lightdm -y
sudo reboot
```

## 5. Install ROS Noetic

### Add ROS Repository

``` bash
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' > /etc/apt/sources.list.d/ros-latest.list"
```

### Add Keys

``` bash
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### Install ROS

``` bash
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

## 6. Initialize ROS

``` bash
sudo rosdep init
rosdep update
```

## 7. Add ROS to Environment

``` bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 8. Create Catkin Workspace

``` bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 9. Install SR-L6 Robot Driver (Example)

``` bash
cd ~/catkin_ws/src
git clone https://github.com/savya-robotics/sr-l6-ros-driver.git
cd ~/catkin_ws
catkin_make
```

## 10. Install ROS Packages

``` bash
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-industrial-core ros-noetic-moveit ros-noetic-rviz -y
```

## 11. Launch Robot Driver

``` bash
roslaunch sr_l6_driver bringup.launch
```

## 12. Basic Commands

### Enable robot

``` bash
rosservice call /sr_l6/enableRobot
```

### Disable robot

``` bash
rosservice call /sr_l6/disableRobot
```

### Read joint states

``` bash
rostopic echo /sr_l6/joint_states
```

### Send joint command

``` bash
rostopic pub /sr_l6/joint_cmd std_msgs/Float64MultiArray "data: [0.1, 0.5, -0.3, 1.0, 0.2, 0]"
```
