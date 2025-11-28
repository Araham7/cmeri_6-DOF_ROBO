
# Full Installation Guide: Ubuntu Server + XFCE + ROS + Savya SR-L6 Arm

<!-- *(With Embedded Official Links and Detailed Explanations)* -->

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

1.  Open **Rufus** - A utility that helps format and create bootable USB drives
2.  Select Ubuntu Server ISO - Choose the downloaded Ubuntu Server image file
3.  Start writing to USB - This process will erase the USB drive and make it bootable

### On Linux (dd method)

```bash
# 'dd' is a disk duplication command that copies the ISO byte-for-byte to the USB
# if=ubuntu-server.iso: input file (your downloaded ISO)
# of=/dev/sdX: output file (replace X with your USB drive letter - be careful!)
# status=progress: shows the copy progress
sudo dd if=ubuntu-server.iso of=/dev/sdX status=progress
```

⚠️ Warning: Make sure you use the correct USB device (/dev/sdX) as this will erase all data on that device.

---

# 🟦 2. Install Ubuntu Server

· Choose English - Select English as the installation language
· Connect WiFi/Ethernet - Ensure internet connection for package downloads
· Enable OpenSSH Server - Allows remote access via SSH
· Reboot after installation - System will restart to complete the installation

---

# 🟦 3. Update System

```bash
# Update package list from repositories
sudo apt update

# Upgrade all installed packages to latest versions
# -y flag automatically answers "yes" to prompts
sudo apt upgrade -y
```

---

# 🟦 4. Install XFCE Lightweight GUI

```bash
# Install XFCE desktop environment and additional useful components
sudo apt install xfce4 xfce4-goodies -y

# Install lightdm display manager for graphical login
sudo apt install lightdm -y

# Reboot to start the graphical interface
sudo reboot
```

> Note: After reboot, you'll see a graphical login screen instead of the command line.

---

# 🟦 5. Install Real-time Kernel (Optional but Recommended for Robotics)

```bash
# Install the Linux real-time kernel for improved timing precision
# This is important for robotics applications requiring deterministic timing
sudo apt install linux-image-rt-5.4.0-rt20-generic linux-headers-rt-5.4.0-rt20-generic -y

# Update GRUB bootloader to recognize the new kernel
sudo update-grub

# Reboot to use the real-time kernel
sudo reboot

# Verify real-time kernel is running
uname -r
# Should show something with "rt" in the name
```

---

# 🟦 6. Install ROS Noetic

### `Add ROS Repository`

```bash
# Add ROS package repository to system's software sources
# $(lsb_release -sc) automatically detects your Ubuntu version (focal for 20.04)
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' > /etc/apt/sources.list.d/ros-latest.list"
```

### `Add Keys`

```bash
# Install curl if not already installed (for downloading files)
sudo apt install curl -y

# Add ROS cryptographic key to verify package authenticity
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### `Install ROS`

```bash
# Update package list to include new ROS repository
sudo apt update

# Install ROS Noetic desktop full version (includes GUI tools, simulators, etc.)
sudo apt install ros-noetic-desktop-full -y
```

---

# 🟦 7. Initialize rosdep

```bash
# Initialize rosdep (ROS dependency management tool)
# This sets up system dependencies for ROS packages
sudo rosdep init

# Update rosdep database to latest package dependencies
rosdep update
```

---

# 🟦 8. Add ROS to .bashrc

```bash
# Add ROS environment setup to bashrc so it loads automatically on terminal startup
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# Apply changes to current terminal session
source ~/.bashrc
```

Explanation: The .bashrc file runs every time you open a new terminal, ensuring ROS environment variables are always set.

---

# 🟦 9. Create Catkin Workspace

```bash
# Create catkin workspace directory structure
# catkin is ROS's build system, similar to CMake but specialized for ROS
mkdir -p ~/catkin_ws/src

# Navigate to workspace root
cd ~/catkin_ws

# Build the workspace (compiles packages in src folder)
catkin_make

# Add workspace setup to bashrc for automatic sourcing
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Apply changes to current terminal
source ~/.bashrc
```

---

# 🟦 10. Install Savya SR-L6 ROS Driver

```bash
# Navigate to source directory of catkin workspace
cd ~/catkin_ws/src

# Clone the SR-L6 ROS driver repository (replace with official repo if different)
git clone https://github.com/savya-robotics/sr-l6-ros-driver.git

# Return to workspace root and build the new driver package
cd ~/catkin_ws
catkin_make
```

---

# 🟦 11. Install Required ROS Packages

```bash
# Install additional ROS packages needed for robot control:
# ros-control: ROS controller infrastructure
# ros-controllers: Common robot controllers
# industrial-core: Industrial robot support
# moveit: Motion planning framework
# rviz: 3D visualization tool
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-industrial-core ros-noetic-moveit ros-noetic-rviz -y
```

---

# 🟦 12. Launch Robot Driver

```bash
# Launch the SR-L6 robot driver
# This starts the ROS nodes that communicate with the physical robot
roslaunch sr_l6_driver bringup.launch
```

---

# 🟦 13. Basic Robot Commands

### `Enable Robot`

```bash
# Send service call to enable robot motors and control
# This powers on the robot arm and prepares it for movement
rosservice call /sr_l6/enableRobot
```

### `Disable Robot`

```bash
# Send service call to disable robot motors
# This safely powers off the robot arm
rosservice call /sr_l6/disableRobot
```

### `Read Joint States`

```bash
# Display real-time joint position, velocity, and effort data
# Useful for monitoring current robot state
rostopic echo /sr_l6/joint_states
```

### `Publish Joint Command`

```bash
# Send joint position commands to the robot
# The array contains target positions for each joint in radians
# Format: [joint1, joint2, joint3, joint4, joint5, joint6]
rostopic pub /sr_l6/joint_cmd std_msgs/Float64MultiArray "data: [0.1, 0.5, -0.3, 1.0, 0.2, 0]"
```

---

# 🟦 **Useful ROS Documentation Links**

* **ROS Tutorials:** [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
* **MoveIt Motion Planning:** [https://moveit.ros.org/](https://moveit.ros.org/)
* **RViz Visualization:** [http://wiki.ros.org/rviz](http://wiki.ros.org/rviz)

---

# 🟩 **Setup Completed Successfully** 🎉

You now have a complete Ubuntu Server system with:

* XFCE desktop environment for graphical interface
* Real-time kernel for improved robotics performance
* ROS Noetic framework for robot software development
* Savya SR-L6 driver for controlling the 6-axis robot arm
* Essential ROS packages including MoveIt for motion planning and RViz for visualization

The system is ready for developing and running robotics applications with the SR-L6 robotic arm.
