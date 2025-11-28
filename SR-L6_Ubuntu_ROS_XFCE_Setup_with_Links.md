# **Optimal Setup for Controlling Savya Robotics SR-L6 6-DoF Robot Arm**

### **Using Ubuntu Server + XFCE GUI + ROS Noetic**

### *(Full Step-by-Step Installation Guide with Detailed Comments + Embedded Official Links)*

---

# 🔗 **Official Downloads (Click to Open)**

* **Ubuntu Server 20.04 LTS**
  👉 [https://ubuntu.com/download/server](https://ubuntu.com/download/server)

* **Rufus (Bootable USB Creator for Windows)**
  👉 [https://rufus.ie/](https://rufus.ie/)

* **ROS Noetic Official Installation Guide**
  👉 [http://wiki.ros.org/noetic/Installation/Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)

* **Savya Robotics – Official Website**
  👉 [https://savyarobotics.com/](https://savyarobotics.com/)

---

# 🟦 **1. Create Ubuntu Server Bootable USB**

## **➡️ On Windows — Using Rufus**

1. Download Rufus from the official link.
2. Insert USB Drive.
3. Choose your **Ubuntu Server ISO file**.
4. Keep Partition Type = *GPT*, File System = *FAT32*.
5. Click **START**.

## **➡️ On Linux — Using dd (VERY Powerful Command)**

```bash
sudo dd if=ubuntu-server.iso of=/dev/sdX status=progress
```

### **💬 Comment:**

* `if=` → Input ISO file.
* `of=` → Output USB device (⚠️ VERY IMPORTANT — Wrong selection wipes entire disk).
* `status=progress` → Shows progress while writing.

---

# 🟦 **2. Install Ubuntu Server**

During installation:

* Select **English**
* Connect to **WiFi/Ethernet**
* Enable **OpenSSH Server** (important for remote usage)
* Reboot after installation

---

# 🟦 **3. Update the System**

```bash
sudo apt update
sudo apt upgrade -y
```

### **💬 Comment:**

* `apt update` → Refresh package list.
* `apt upgrade` → Install newest versions of all packages.
* `-y` → Auto yes, no need to press Y.

---

# 🟦 **4. Install XFCE Lightweight GUI (Best for Server)**

```bash
sudo apt install xfce4 xfce4-goodies -y
sudo apt install lightdm -y
sudo reboot
```

### **💬 Comment:**

* `xfce4` → Fastest & lightest desktop environment.
* `xfce4-goodies` → Extra tools for better UI.
* `lightdm` → Display manager to show login screen.
* `reboot` → Required to activate GUI.

---

# 🟦 **5. Install ROS Noetic (For Ubuntu 20.04)**

### **▶ Add ROS repository**

```bash
sudo sh -c "echo 'deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main' > /etc/apt/sources.list.d/ros-latest.list"
```

### **💬 Comment:**

* Adds official ROS package server.

### **▶ Add ROS Key**

```bash
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### **💬 Comment:**

* Needed so Ubuntu trusts ROS packages.

### **▶ Install ROS Desktop-Full**

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```

### **💬 Comment:**

* Installs RViz, Gazebo, MoveIt, ROS tools.
* Full robotics development package.

---

# 🟦 **6. Initialize rosdep (Mandatory)**

```bash
sudo rosdep init
rosdep update
```

### **💬 Comment:**

* `rosdep` installs system dependencies required by ROS packages.
* Must be run once after installing ROS.

---

# 🟦 **7. Add ROS Environment to .bashrc**

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **💬 Comment:**

* Automatically activates ROS every time terminal opens.

---

# 🟦 **8. Create Catkin Workspace**

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **💬 Comment:**

* `catkin_ws` = ROS project workspace.
* `src/` = where packages will be cloned.
* `catkin_make` = builds workspace.

---

# 🟦 **9. Install Savya SR-L6 ROS Driver**

### *(Example Repo — Replace if Savya Provides Official Repo)*

```bash
cd ~/catkin_ws/src
git clone https://github.com/savya-robotics/sr-l6-ros-driver.git
cd ~/catkin_ws
catkin_make
```

### **💬 Comment:**

* Clones robot driver package.
* Rebuild workspace to integrate new package.

---

# 🟦 **10. Install Required ROS Packages for Robot Control**

```bash
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-industrial-core ros-noetic-moveit ros-noetic-rviz -y
```

### **💬 Comment:**

* `ros-control` → Real-time joint control
* `industrial-core` → Industrial robot communication
* `MoveIt` → Motion planning & IK
* `RViz` → 3D visualization

---

# 🟦 **11. Launch Robot Driver**

```bash
roslaunch sr_l6_driver bringup.launch
```

### **💬 Comment:**

* Starts communication between ROS & SR-L6 robot.

---

# 🟦 **12. Basic Robot Commands**

## **➡ Enable Robot**

```bash
rosservice call /sr_l6/enableRobot
```

## **➡ Disable Robot**

```bash
rosservice call /sr_l6/disableRobot
```

## **➡ Read Joint States**

```bash
rostopic echo /sr_l6/joint_states
```

## **➡ Send Joint Commands**

```bash
rostopic pub /sr_l6/joint_cmd std_msgs/Float64MultiArray "data: [0.1, 0.5, -0.3, 1.0, 0.2, 0]"
```

### **💬 Comment:**

* Command sends target angles in **radians** for all 6 joints.

---

# 🟦 **Useful ROS Documentation Links**

* **ROS Tutorials:** [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
* **MoveIt Motion Planning:** [https://moveit.ros.org/](https://moveit.ros.org/)
* **RViz Visualization:** [http://wiki.ros.org/rviz](http://wiki.ros.org/rviz)

---

# 🟩 **Setup Completed Successfully** 🎉

You now have:

* Ubuntu Server with XFCE GUI
* Full ROS Noetic Installed
* Catkin Workspace Ready
* SR-L6 Robot Driver Integrated

Your system is now ready for **industrial-level robotic arm control, automation, motion planning, and ROS experiments**.

If you want I can also create:
✅ PDF version
✅ Flowchart diagrams
✅ Architecture diagram
✅ ROS Node graph (rqt_graph)

Just tell me, brother ❤️
