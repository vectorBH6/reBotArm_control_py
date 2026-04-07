# 🦾 reBotArm Control: Python Robotic Arm Control Library

<p align="center">
    <a href="./LICENSE">
        <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
    </a>
    <img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python Version">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Ubuntu-orange.svg" alt="Platform">
    <img src="https://img.shields.io/badge/Framework-Pinocchio-yellow.svg" alt="Pinocchio">
</p>

<p align="center">
  <strong>6-DOF Robotic Arm · Multi-Motor Support · Kinematics Solver · Trajectory Planning · Fully Open Source</strong>
</p>

<p align="center">
  <strong>
    <a href="./README_zh.md">简体中文</a> &nbsp;|&nbsp;
    <a href="./README.md">English</a> &nbsp;|&nbsp;
    <a href="./README_JP.md">日本語</a>&nbsp;|&nbsp;
    <a href="./README_Fr.md">français</a>&nbsp;|&nbsp;
    <a href="./README_es.md">Español</a>
  </strong>
</p>

---

## 📖 Introduction

**reBotArm Control** is a Python control library for the reBot Arm B601 robotic arm, providing a complete solution from low-level motor control to high-level kinematics computation.

### ✨ Core Features

- 🦾 **Multi-Motor Support** — Damiao, MyActuator, RobStride three motor brands
- 🎯 **Three Control Modes** — MIT, POS_VEL, VEL for different application scenarios
- 🧮 **Kinematics Solver** — Forward/Inverse kinematics based on Pinocchio
- 🛤️ **Trajectory Planning** — SE(3) geodesic trajectory + CLIK tracking
- 🔧 **Flexible Configuration** — YAML configuration file for quick hardware adaptation

---

## ⚙️ Quick Start

### Requirements

| Item | Requirement |
|------|-------------|
| **Python** | 3.10+ |
| **Operating System** | Ubuntu 22.04+ |
| **Communication Interface** | USB2CAN Serial Bridge or CAN Interface |

### Installation Steps

#### Step 1. Install uv (if not installed)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

#### Step 2. Sync Environment (Install All Dependencies)

```bash
uv sync
```

:::tip
`uv sync` will automatically create a virtual environment (if it doesn't exist) and install all dependencies according to `pyproject.toml` and `uv.lock`.
:::

---

## 🔌 Hardware Configuration

### Default: Damiao USB2CAN Serial Bridge

reBot Arm B601-DM uses the Damiao USB2CAN serial bridge module by default.

**Hardware Connection**:
1. Connect the USB2CAN module to your computer via USB cable
2. The system will automatically recognize it as `/dev/ttyACM0` device

**Configuration Verification**:
```bash
# Check device
ls /dev/ttyACM0

# Scan motors
motorbridge-cli scan --vendor damiao --transport dm-serial \
    --serial-port /dev/ttyACM0 --serial-baud 921600
```

### Optional: Standard CAN Interface

Using other USB-CAN adapters (CANable, PCAN, etc.):

```bash
# Start CAN interface
sudo ip link set can0 up type can bitrate 500000

# Verify interface
ip -details link show can0
```

### Motor Brand Configuration

| Motor Brand | Transmission | Configuration | Baud Rate |
|-------------|--------------|---------------|-----------|
| **Damiao** | Serial Bridge | `dm-serial` | 921600 |
| **Damiao** | CAN Interface | `socketcan` | 500000 |
| **MyActuator** | CAN Interface | `socketcan` | 500000 |
| **RobStride** | CAN Interface | `socketcan` | 500000 |

:::tip
- For Damiao motors using serial bridge, must set `--transport dm-serial`
- Feedback ID rule: `feedback_id = motor_id + 0x10`
:::

---

## 📁 Project Structure

```
reBotArm_control_py/
├── config/                     # Configuration files
│   └── robot.yaml              # Joint parameter configuration
├── example/                    # Example programs
│   ├── Debug Tools/
│   │   ├── 0x01damiao_text.py      # Single motor console
│   │   └── 2_zero_and_read.py      # Zero calibration
│   ├── Position Control/
│   │   ├── 3_mit_control.py        # MIT control
│   │   └── 4_pos_vel_control.py    # POS_VEL control
│   ├── Kinematics Tests/
│   │   ├── 5_fk_test.py            # Forward kinematics
│   │   └── 6_ik_test.py            # Inverse kinematics
│   ├── Real Machine Control/
│   │   ├── 7_arm_ik_control.py     # IK real-time control
│   │   └── 8_arm_traj_control.py   # Trajectory planning
│   └── sim/                    # Simulation tools
├── reBotArm_control_py/        # Core library
│   ├── actuator/               # Actuator module
│   ├── kinematics/             # Kinematics module
│   ├── controllers/            # Controller module
│   └── trajectory/             # Trajectory planning module
├── urdf/                       # URDF model
└── README.md
```

---

## 🎮 Example Programs

### Debug Tools

#### 1️⃣ Single Motor Console (`0x01damiao_text.py`)

Direct motorbridge SDK single motor testing with three control modes.

**Usage**:
```bash
uv run python example/0x01damiao_text.py
```

**Interactive Commands**:
| Command | Description |
|---------|-------------|
| `mit <pos_deg> [vel kp kd tau]` | MIT mode |
| `posvel <pos_deg> [vlim]` | POS_VEL mode |
| `vel <vel_rad_s>` | Velocity mode |
| `enable` / `disable` | Enable/Disable |
| `set_zero` | Set zero position |
| `state` | View state |

---

#### 2️⃣ Zero Calibration & Angle Monitor (`2_zero_and_read.py`)

Automatically set all joint zeros and display joint angles in real-time.

**Usage**:
```bash
uv run python example/2_zero_and_read.py
```

---

### Position Control

#### 3️⃣ MIT Spring-Damper Control (`3_mit_control.py`)

Multi-joint MIT mode position control with real-time PID adjustment.

**Input Format**:
```
<joint1_deg> <joint2_deg> ... <jointN_deg> [kp] [kd]
```

**Example**:
```bash
uv run python example/3_mit_control.py
> 0 0 0 0 0 0          # All joints to zero
> 10 -20 30 -40 50 60  # Set specific angles
> state                # View state
> q                    # Quit
```

---

#### 4️⃣ POS_VEL Position-Velocity Control (`4_pos_vel_control.py`)

Position-velocity dual-loop PI control.

**Input Format**:
```
<joint1_deg> <joint2_deg> ... <jointN_deg> [vlim]
```

**Usage**:
```bash
uv run python example/4_pos_vel_control.py
```

---

### Kinematics Tests

#### 5️⃣ Forward Kinematics Test (`5_fk_test.py`)

Calculate end-effector pose from joint angles.

**Input**: 6 joint angles (degrees)

**Output**:
- End-effector position (X, Y, Z) — Unit: meters
- Rotation matrix (3×3)
- Euler angles (Roll/Pitch/Yaw) — Unit: degrees

**Example**:
```bash
uv run python example/5_fk_test.py
> 0 0 0 0 0 0
> 45 -30 15 -60 90 180
```

---

#### 6️⃣ Inverse Kinematics Test (`6_ik_test.py`)

Solve joint angles from desired end-effector pose.

**Input Format**:
- Position only: `<x> <y> <z>` (meters)
- Position + Orientation: `<x> <y> <z> <roll> <pitch> <yaw>` (degrees)

**Example**:
```bash
uv run python example/6_ik_test.py
> 0.25 0.0 0.15              # Position only
> 0.25 0.0 0.15 0 0 0        # Position + Orientation
```

---

### Real Machine Control

#### 7️⃣ IK Real-time Control (`7_arm_ik_control.py`)

Real-time end-effector control based on IK solver.

**Interactive Commands**:
| Command | Description |
|---------|-------------|
| `x y z [roll pitch yaw]` | Target end-effector pose |
| `state` | View current/target state |
| `pos` | Current end-effector position |
| `q/quit/exit` | Quit |

**Usage**:
```bash
uv run python example/7_arm_ik_control.py
> 0.3 0.0 0.2
> 0.3 0.1 0.25 0 0.5 0
```

---

#### 8️⃣ Trajectory Planning Control (`8_arm_traj_control.py`)

SE(3) geodesic trajectory planning + CLIK tracking.

**Input Format**:
```
x y z [roll pitch yaw] [duration]
```

**Parameters**:
- `x, y, z`: Target position (meters)
- `roll, pitch, yaw`: Target orientation (radians)
- `duration`: Movement duration (seconds), default 2.0s

**Usage**:
```bash
uv run python example/8_arm_traj_control.py
> 0.3 0.0 0.3 0 0.4 0 2.0
```

---

## 📚 Core Module API

### RobotArm Main Control Class

```python
from reBotArm_control_py.actuator import RobotArm
import numpy as np

arm = RobotArm("config/robot.yaml")
arm.connect()
arm.enable()
arm.set_zero()

# MIT Mode
arm.mode_mit()
arm.mit(pos=np.zeros(6), kp=np.ones(6)*100, kd=np.ones(6)*5)

# POS_VEL Mode
arm.mode_pos_vel()
arm.pos_vel(pos=np.zeros(6))

arm.disconnect()
```

### Kinematics Module

```python
from reBotArm_control_py.kinematics import (
    load_robot_model,
    compute_fk,
    compute_ik,
)

# Forward Kinematics
model = load_robot_model()
position, rotation, homogeneous = compute_fk(model, q)

# Inverse Kinematics
result = compute_ik(
    q_init=np.zeros(6),
    target_pos=np.array([0.3, 0.0, 0.2]),
)
```

### Advanced Controllers

```python
from reBotArm_control_py.controllers import ArmIK, ArmTraj

# IK Controller
arm_ik = ArmIK(arm)
arm_ik.start()
arm_ik.move_to_ik(x=0.3, y=0.1, z=0.4)
arm_ik.end()

# Trajectory Controller
arm_traj = ArmTraj(arm)
arm_traj.start()
arm_traj.move_to_traj(x=0.3, y=0.0, z=0.3, duration=2.0)
arm_traj.end()
```

---

## 🎯 Control Mode Comparison

| Mode | Principle | Application |
|------|-----------|-------------|
| **MIT** | Torque = kp×pos_err + kd×vel_err | Compliant control, impedance control |
| **POS_VEL** | PI position loop + PI velocity loop | Precise position control |
| **VEL** | Direct velocity command | Velocity mode applications |

---

## 🙌 References & Acknowledgments

### Ecosystem & Software Support
*   **[Pinocchio](https://stack-of-tasks.github.io/pinocchio/)** — Rigid body dynamics library
*   **[motorbridge](https://github.com/damiao-robot/motorbridge)** — Motor SDK

### Core Hardware Partners
*   **[Damiao Technology](https://www.damiaokeji.com/)**
*   **[MyActuator](https://myactuator.com/)**
*   **[RobStride](https://robstride.com/)**

---

## 📄 License

This project is open source under the **MIT License**.

---

## ☎ Contact Us

- **Technical Support**: [Submit Issue](https://github.com/vectorBH6/reBotArm_control_py/issues)
- **Repository**: [GitHub](https://github.com/vectorBH6/reBotArm_control_py)

---

<p align="center">
  <strong>🌟 If this project helps you, please give us a Star!</strong>
</p>
