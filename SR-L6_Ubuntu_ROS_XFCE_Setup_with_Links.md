# Full Installation Guide: Ubuntu Server + XFCE + ROS + Savya SR-L6 Arm

*(With Embedded Official Links)*

------------------------------------------------------------------------

# 🔗 Official Downloads

### **Ubuntu Server 20.04 LTS**

👉 https://ubuntu.com/download/server

### **Rufus (Bootable USB creator for Windows)**

👉 https://rufus.ie/

### **ROS Noetic Official Installation Guide**

👉 http://wiki.ros.org/noetic/Installation/Ubuntu

### **Savya Robotics (Official Website)**

👉 https://savyarobotics.com/

------------------------------------------------------------------------

# 🟦 1. Create Ubuntu Server Bootable USB

### On Windows (Rufus)

1.  Open **Rufus**\
2.  Select Ubuntu Server ISO\
3.  Start writing to USB

### On Linux (dd method)

``` bash
sudo dd if=ubuntu-server.iso of=/dev/sdX status=progress
```

------------------------------------------------------------------------

# 🟦 2. Install Ubuntu Server

-   Choose **English**\
-   Connect WiFi/Ethernet\
-   Enable **OpenSSH Server**\
-   Reboot after installation

------------------------------------------------------------------------

# 🟦 3. Update System

``` bash
sudo apt update
sudo apt upgrade -y
```

------------------------------------------------------------------------

# 🟦 4. Install XFCE Lightweight GUI

``` bash
sudo apt install xfce4 xfce4-goodies -y
sudo apt install lightdm -y
sudo reboot
```

------------------------------------------------------------------------

# 🟦 5. Install ROS Noetic

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

------------------------------------------------------------------------

# 🟦 6. Initialize rosdep

``` bash
sudo rosdep init
rosdep update
```

------------------------------------------------------------------------

# 🟦 7. Add ROS to .bashrc

``` bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

------------------------------------------------------------------------

# 🟦 8. Create Catkin Workspace

``` bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

------------------------------------------------------------------------

# 🟦 9. Install Savya SR-L6 ROS Driver

*(Example repository -- replace with official if provided)*

``` bash
cd ~/catkin_ws/src
git clone https://github.com/savya-robotics/sr-l6-ros-driver.git
cd ~/catkin_ws
catkin_make
```

------------------------------------------------------------------------

# 🟦 10. Install Required ROS Packages

``` bash
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-industrial-core ros-noetic-moveit ros-noetic-rviz -y
```

------------------------------------------------------------------------

# 🟦 11. Launch Robot Driver

``` bash
roslaunch sr_l6_driver bringup.launch
```

------------------------------------------------------------------------

# 🟦 12. Basic Robot Commands

### Enable Robot

``` bash
rosservice call /sr_l6/enableRobot
```

### Disable Robot

``` bash
rosservice call /sr_l6/disableRobot
```

### Read Joint States

``` bash
rostopic echo /sr_l6/joint_states
```

### Publish Joint Command

``` bash
rostopic pub /sr_l6/joint_cmd std_msgs/Float64MultiArray "data: [0.1, 0.5, -0.3, 1.0, 0.2, 0]"
```

------------------------------------------------------------------------

# 🟦 Useful ROS Documentation Links

### ROS Tutorials

👉 http://wiki.ros.org/ROS/Tutorials

### MoveIt Motion Planning

👉 https://moveit.ros.org/

### RViz Visualization

👉 http://wiki.ros.org/rviz

------------------------------------------------------------------------

# 🟩 Setup Completed

You can now control the SR-L6 6-axis robot arm using ROS, MoveIt, and
Ubuntu Server with XFCE GUI.
