<!--
Istallation of `ubuntu 24.04 lts(noble numbat)`
### The latest LTS version of Ubuntu, for desktop PCs and laptops. LTS stands for long-term support — which means five years of free security and maintenance updates, extended up to 15 years with Ubuntu Pro.
# 1. [Ubuntu 24.04 `lts`(noble numbat)](https://ubuntu.com/download/desktop)

ROS2-Jazzy-Installation-Manual
# 2. [Installation of `ROS2-jazzy` guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)


Gazebo-Installation-Manual
# 3. [Installation of `Gazibo-harmonic(LTS)` guide](https://gazebosim.org/docs/harmonic/install_ubuntu/)


>[CLICK ME(To view the importent Image)](https://gazebosim.org/docs/latest/ros_installation/)
 -->




## Deterministic Robot Control Using Ubuntu Server + PREEMPT_RT + Python

---

## 1. Problem Statement

When running a loop-based program on **Windows**, the total execution time and loop intervals vary every time the program is executed.  
This behavior becomes critical when the program is used for **real-time control**, such as controlling a **6-DOF industrial robot** via Ethernet.

### Observed Issue
- Same program → different execution times on every run
- Loop interval is not constant
- Causes instability in robot motion

---

## 2. System Context (Important)

- The **6-DOF robot has its own dedicated CPU and RTOS**
- The PC **does NOT control motors directly**
- The PC sends motion commands to the robot via **Ethernet**
- Control is done using a **Python API only**

> Therefore, the real-time issue is **not inside the robot**,  
> but on the **PC side (OS + Python + network scheduling)**.

---

## 3. Root Cause Analysis

### Why Windows (or normal Linux) fails

Windows and standard Linux distributions are **NOT real-time operating systems**.

Main reasons:
- OS scheduler jitter
- Background processes
- Python interpreter overhead
- Inaccurate `time.sleep()`
- CPU frequency scaling (dynamic GHz changes)
- Network stack buffering

Result:
> **Python + Windows / normal Linux = NON-DETERMINISTIC timing**

---

## 4. What Is Actually Required?

The requirement is **deterministic timing**, meaning:
- Same loop interval every time
- Minimal jitter
- Stable behavior even years later

This is called **Soft Real-Time Control** on the PC side.

---

## 5. Recommended Solution (Industry-Practical)

### ✅ Best Practical Solution
**Ubuntu Server + PREEMPT_RT + Python**

This is widely used in:
- Industrial robotics
- CNC controllers
- Motion control PCs
- Robot command streaming systems

---

## 6. Why PREEMPT_RT?

PREEMPT_RT is a **Real-Time Linux kernel** that:
- Makes the Linux scheduler deterministic
- Reduces latency and jitter
- Allows real-time thread priorities
- Improves network packet scheduling

> The PC still remains **soft real-time**,  
> but jitter is reduced **10–20× compared to Windows**.

---

## 7. Recommended Architecture
```bash
Python Application (PC, RT Linux)
|
| Ethernet (TCP/UDP)
|
Robot Controller (Dedicated CPU + RTOS)
```


- PC = Command planner
- Robot controller = Real-time executor

---

## 8. OS Installation Choice

### ✅ Ubuntu Server (Recommended)
Reasons:
- No GUI → fewer background processes
- Lower latency
- Better real-time stability
- Preferred for industrial setups

### Recommended Version
```bash
Ubuntu Server 22.04 LTS
```

---

## 9. Installing PREEMPT_RT Kernel

### Step 1: Update system
```bash
sudo apt update
sudo apt upgrade
```
# Step 2: Install RT kernel
```bash
sudo apt install linux-image-rt-amd64 linux-headers-rt-amd64
```

# Step 3: Reboot
```bash
reboot
```

# Step 4: Verify
```bash
uname -a
```

# Expected output must include:
```
PREEMPT_RT
```

# ✅ System is now running a Real-Time Linux kernel.

# 10. CPU & Power Optimization (CRITICAL)
## 10.1 Disable CPU Frequency Scaling


# What is CPU Frequency Scaling?

## Modern CPUs automatically change speed (GHz) to save power:

- Low load → low frequency
- High load → high frequency
This causes timing jitter in control loops.

### Solution: Lock CPU to Performance Mode
```bash
sudo apt install cpufrequtils
sudo cpufreq-set -g performance
```

### Verify:
```bash
cpufreq-info
```

## 10.2 Disable Sleep & Power Saving
```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

# 11. Running Python with Real-Time Priority

Run your Python control script using FIFO scheduling:
```bash
sudo chrt -f 90 python3 robot_control.py
```
- `-f` → FIFO real-time scheduler
- `90` → High priority (max is 99)

# 12. Python Timing Best Practices
❌ Do NOT use `time.sleep()`
```bash
time.sleep(0.001)
```
# ✅ Use deterministic loop
```bash
import time

PERIOD_NS = 1_000_000  # 1 ms = 1000 Hz
next_t = time.perf_counter_ns()

while True:
    send_robot_command()

    next_t += PERIOD_NS
    while time.perf_counter_ns() < next_t:
        pass
```
* ⚠️ CPU usage will be high
* ✅ Timing will be stable

# 13. Ethernet Optimization (Very Important)
Recommended settings:

* PC ↔ Robot direct Ethernet cable
* Disable Wi-Fi
* Use static IP (no DHCP)
* Disable Nagle’s algorithm
* Python socket optimization

# Python socket optimization
```
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
```
> If the robot API supports UDP → even better.

# 14. Jitter Comparison (Typical)

| Platform            | Timing Jitter |
| ------------------- | ------------- |
| Windows + Python    | ±1–10 ms      |
| Normal Ubuntu       | ±0.5–2 ms     |
| Ubuntu + PREEMPT_RT | ±50–100 µs    |

# 15. Reality Check (Important)

* PC + Python ≠ Hard Real-Time
* Robot controller IS real-time
* PC should only stream commands or trajectories
> This setup is standard industry practice.

# 16. Best Practice (If API Supports It)
`Prefer:`
* Trajectory upload
* Time-stamped commands
* Servo / streaming modes
> This shifts timing responsibility fully to the robot controller.

# 17. Final Recommendation

Best and future-proof setup:
```
Ubuntu Server 22.04
+ PREEMPT_RT Kernel
+ CPU Performance Mode
+ High-priority Python Loop
```

### ✅ Python API can still be used
### ❌ Windows should be avoided

# 18. Next Steps (Optional)
For further optimization:

* CPU core isolation (isolcpus)
* Disable Turbo Boost
* Bind Python process to a single CPU core
Measure latency using cyclictest


# 19. Conclusion (One Line)

**Ubuntu Server + PREEMPT_RT + Python** provides the most stable, repeatable, and industry-accepted solution for Ethernet-based 6-DOF robot control using a Python API, ensuring low jitter, predictable timing, and long-term reliability compared to Windows or non-real-time operating systems.
