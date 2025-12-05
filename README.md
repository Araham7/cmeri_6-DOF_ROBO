<!-- Istallation of `ubuntu 24.04 lts(noble numbat)` -->
### The latest LTS version of Ubuntu, for desktop PCs and laptops. LTS stands for long-term support — which means five years of free security and maintenance updates, extended up to 15 years with Ubuntu Pro.
# 1. [Ubuntu 24.04 `lts`(noble numbat)](https://ubuntu.com/download/desktop)

<!-- ROS2-Jazzy-Installation-Manual -->
# 2. [Installation of `ROS2-jazzy` guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)


<!-- Gazebo-Installation-Manual -->
# 3. [Installation of `Gazibo-harmonic(LTS)` guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)


>[CLICK ME(To view the importent Image)](https://gazebosim.org/docs/latest/ros_installation/)

<!--
# What is Linux?

Linux is an open-source, Unix-like operating system (OS) kernel that serves as the foundation for many operating systems. It was initially created by **Linus Torvalds** in 1991 and has since evolved into one of the most widely used operating systems worldwide.

## Key points about Linux:
- **Open Source**: Its source code is available for anyone to view, modify, and distribute, making it highly customizable.
- **Kernel**: The Linux "kernel" is the core part of the system that manages hardware resources (such as CPU, memory, and I/O).
- **Distributions**: Linux is packaged in various distributions (distros), such as Ubuntu, Fedora, Debian, and CentOS, which provide different user interfaces, tools, and functionalities.
- **Multi-user, Multi-tasking**: Linux supports multiple users and allows multiple tasks to run simultaneously.
- **Security and Stability**: Linux is known for its strong security features and reliability, making it popular for servers, embedded systems, and even desktops.
- **Command Line Interface (CLI)**: While many distros offer graphical interfaces, Linux traditionally relies on the command line for system management, providing great power and flexibility.

---

# What is Real-Time Linux (RT Linux)?

**Real-Time Linux (RT Linux)** refers to a version of the Linux kernel that has been modified to support **real-time operations**, where tasks must meet strict timing constraints. In real-time systems, there are guarantees that certain tasks will complete within a fixed, predictable amount of time.

## Key points about Real-Time Linux:
- **Real-time requirements**: Unlike regular Linux, which is optimized for throughput and fairness, real-time systems prioritize predictable, timely task execution. This is critical in applications like industrial automation, robotics, aerospace, medical devices, and telecommunications.
- **Deterministic behavior**: In an RT system, the latency—the time it takes for a task to be executed after a trigger event—should be minimal and predictable. This makes RT Linux suitable for systems where timing is crucial, and delayed responses can be catastrophic.
- **Preemption**: In Real-Time Linux, the kernel is modified to reduce latency by allowing tasks to preempt the CPU quickly. This is achieved by making the kernel more preemptive (meaning it can interrupt the execution of lower-priority tasks in favor of higher-priority ones).
- **Real-Time Patches**: To turn standard Linux into a real-time system, various patches, such as **PREEMPT-RT** (a set of patches), can be applied. These patches enhance the real-time capabilities of Linux by reducing non-deterministic behavior and minimizing interrupt latency.
- **Applications**: RT Linux is used in scenarios where predictable behavior is essential. For example:
  - **Automated manufacturing**: Precise control of machines.
  - **Medical equipment**: Real-time monitoring and actuation.
  - **Robotics**: Timely sensor data processing and control responses.
  - **Telecommunications**: Network equipment that needs to handle traffic without delays.

---

# Differences Between Standard Linux and Real-Time Linux:
1. **Scheduling**: 
   - In standard Linux, the scheduler aims to fairly allocate CPU time among all running processes, leading to non-deterministic execution times.
   - In RT Linux, the scheduler is designed to ensure tasks meet deadlines, often using priority-based scheduling.

2. **Interrupt Handling**: 
   - Standard Linux can have significant delays in interrupt handling because of its general-purpose design.
   - RT Linux reduces interrupt handling latency, making it more predictable and responsive to external events.

3. **Preemption**:
   - Standard Linux uses preemption, but it's not designed to guarantee strict timing constraints.
   - RT Linux provides more aggressive preemption to ensure tasks can meet deadlines.

4. **Real-Time Kernels**: 
   - The Linux kernel in its original form is not real-time, but with the PREEMPT-RT patches, Linux can be transformed into a real-time system.

---

# Types of Real-Time Linux Systems:
- **Hard Real-Time Systems**: These systems must complete a task within a strict deadline. If a task is not completed on time, the system will fail. Hard real-time systems are typically used in safety-critical applications.
- **Soft Real-Time Systems**: These systems allow some flexibility in meeting deadlines. Missed deadlines may degrade performance but do not necessarily cause failure. Soft real-time systems are used in less critical applications where timing precision is still important but not absolutely strict.

---

In summary, **Linux** is a flexible and powerful OS, and **Real-Time Linux** (RT Linux) extends its capabilities for applications requiring guaranteed response times and predictable behavior.

-->
