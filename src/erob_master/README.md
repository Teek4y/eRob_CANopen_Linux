# eRob Master - CANopen ROS 2 Controller

eRob Master is a ROS 2-based CANopen motor controller used to control motors that support the CANopen protocol via CAN bus. This controller supports position and velocity control, and provides a series of ROS 2 interfaces for motor monitoring and control.

## Features

- CANopen protocol communication support
- Position and velocity control support
- ROS 2 topics and service interfaces
- Real-time monitoring of motor status, position, and velocity
- Support for motor initialization, start, stop, and reset
- Profile parameter settings support (velocity, acceleration, deceleration)

## Installation

### Prerequisites

- ROS 2 (Humble or higher version recommended)
- CAN interface (e.g., Socket-CAN adapter)
- Motors supporting CANopen protocol

### Build and Install


1. Clone repository

   ```bash
   git clone https://github.com/Teek4y/eRob_CANopen_Linux.git
   ```

3. Build

   ```bash
   cd ~/eRob_CANopen_Linux
   colcon build 
   ```

4. Launch node

   ```bash
   source install/setup.bash
   ros2 launch erob_master canopen_ros2.launch.py
   ```

## Configure CAN Interface

### 1. Load CAN modules

```bash
sudo modprobe can
sudo modprobe can_raw
```

### 2. Configure CAN interface (example for can0 with 1Mbps baudrate)

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

### 3. Check CAN interface status

```bash
ip -details link show can0
```

## 说明

### 1.支持功能

多电机PDO配置、使能、错误清除、重置、PPM/PVM/PTM控制

#### 默认PDO配置

TxPDO1: 状态字(0x6041)+实际位置(0x6064) 6Byte

TxPDO2: 实际电流(0x6078)+实际速度(0x606C) 6Byte

RxPDO1: 控制字(0x6040)+目标位置(0x607A) 6Byte

RxPDO2: 控制字(0x6040)+目标速度(0x60FF) 6Byte

RxPDO3: 控制字(0x6040)+目标力矩(0x6071) 6Byte

### 2.消息类型

多电机实际值以 sensor_msgs::msg::JointState 消息类型通过话题 /erob_joint_state_real 发布

   position：电机实际位置
   velocity：电机实际速度
   effort：电机实际电流

此外订阅 sensor_msgs::msg::JointState 消息类型话题 /target_position_cmd 的数据，将目标下发到关节电机。


## Usage

### 1. Launch Node

Launch with default parameters

```bash
ros2 launch erob_master canopen_ros2.launch.py
```


### 2. Launch with custom parameters（不可用）

```bash
ros2 launch erob_master canopen_ros2.launch.py can_interface:=can0 node_id:=2 auto_start:=true
```

## Controlling the Motor

### 1. Position Control

#### Service Control

Control motor position by publishing to service /set_erob_position :

- Joint 6 Move to 90 degrees

```bash
ros2 service call /set_erob_position erob_master/srv/MoveMotor "{node_id: 6, target: 90.0}"
```

#### Topic Control

- Position Control

Control motor position by publishing sensor_msgs::msg::JointState messge to topic /target_position :

```bash
JointState:
   position[]:
      120.0
      180.0
      120.0
      90.0
      100.0
      0.0
      12.0
```

- Velocity Control

Control motor velocity by publishing sensor_msgs::msg::JointState messge to topic /target_velocity :

```bash
JointState:
   velocity[]:
      120.0
      180.0
      120.0
      90.0
      100.0
      0.0
      12.0
```

- Torque Control

Control motor effort by publishing sensor_msgs::msg::JointState messge to topic /target_effort :

```bash
JointState:
   effort[]:
      120.0
      180.0
      120.0
      90.0
      100.0
      0.0
      12.0
```


### 2. Velocity Control

#### Service Control

Control motor velocity by publishing to service /set_erob_velocity :

- Joint 6 Move at 10 degrees/s

```bash
ros2 service call /set_erob_velocity erob_master/srv/MoveMotor "{node_id: 6, target: 10.0}"
```

#### Topic Control

Control motor velocity by publishing to /target_velocity topic:

- Set velocity to 10 degrees/second

todo

### 3. Torque Control

#### Service Control

Control motor torque by publishing to service /set_erob_effort :

- Joint 6 Move with 1000mN

```bash
ros2 service call /set_erob_effort erob_master/srv/MoveMotor "{node_id: 6, target: 1000}"
```

#### Topic Control

Control motor torque by publishing to /target_torque topic:

todo

## Service Interfaces

### 1. Start Motor

```bash
ros2 service call /start_erob erob_master/srv/MotorID "node_id: 1"
```

### 2. Stop Motor

```bash
ros2 service call /stop_erob erob_master/srv/MotorID "node_id: 1"
```

### 3. Reset Motor

```bash
ros2 service call /reset_erob erob_master/srv/MotorID "node_id: 1"
```

It may take twice the command to reset motor successfully.

## Setting Motor Mode

- Set to position mode

```bash
ros2 service call /set_erob_mode erob_master/srv/ConfigureMotor "{node_id: 1, operation_mode: PPM}"
```

- Set to velocity mode

```bash
ros2 service call /set_erob_mode erob_master/srv/ConfigureMotor "{node_id: 2, operation_mode: PVM}"
```

- Set to torque mode

```bash
ros2 service call /set_erob_mode erob_master/srv/ConfigureMotor "{node_id: 2, operation_mode: PTM}"
```




## Monitor Motor Status（不可用）

```bash
ros2 topic echo /erob_status
```

## View Motor Joint State

```bash
ros2 topic echo /erob_joint_state_real
```

msg.position: 各关节实际位置(deg)
msg.velocity: 各关节实际速度(deg/s)
msg.effort:   各关节实际电流(mA)

## Topic List

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
| /target_position | sensor_msgs/msg/JointState | Set target position (deg) |
| /target_velocity | sensor_msgs/msg/JointState | Set target velocity (deg/s) |
| /erob_status | std_msgs/msg/String | Motor status information |
| /erob_joint_state_real | sensor_msgs/msg/JointState | Current position(deg), velocity(deg/s), effort(mA) |

## Service List

| Service Name | Service Type | Description |
| ------------ | ------------ | ----------- |
| /start_erob | erob_master/srv/MotorID | Start motor ("node_id: 1") |
| /stop_erob | erob_master/srv/MotorID | Stop motor ("node_id: 1") |
| /reset_erob | erob_master/srv/MotorID | Reset motor ("node_id: 1") |
| /set_erob_mode | erob_master/srv/ConfigureMotor | Set motor mode ("PPM": position mode, "PVM": velocity mode, "PTM": Torque mode) |
| /set_erob_position | erob_master/srv/MoveMotor | Set motor position ("{node_id: 1, target: 180.0}") |

## Parameter List

| Parameter Name | Type | Default Value | Description |
| -------------- | ---- | ------------- | ----------- |
| can_interface | string | "can0" | CAN interface name |
| node_id | int | 2 | CANopen node ID |
| auto_start | bool | true | Whether to auto-start motor |
| profile_velocity | int | 5 | Profile velocity (degrees/second) |
| profile_acceleration | int | 5 | Profile acceleration (degrees/second²) |
| profile_deceleration | int | 5 | Profile deceleration (degrees/second²) |

## Troubleshooting

### Cannot Connect to CAN Interface

- Check if CAN interface is properly configured: ip -details link show can0
- Ensure CAN interface is up: sudo ip link set up can0
- Verify CAN bus connections are correct

### Motor Not Responding to Commands

- Check if motor power is connected
- Confirm node ID is correct
- Monitor CAN bus communication using candump: candump can0
- Check motor status: ros2 topic echo /erob_status

### Cannot Switch Operation Mode

- Some motors may not support standard CiA402 operation mode switching. In this case, the controller will attempt to work in the current mode.

### Advanced Usage

- Using candump to monitor CAN communication
Install can-utils

```bash
sudo apt-get install can-utils
```

### Monitor CAN bus

```bash
candump can0
```

## License

Apache License 2.0
