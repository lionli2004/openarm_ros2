# OpenArm V10 双臂系统 — 开发配置文档

> **生成日期**: 2026-07-25  
> **维护者**: Lionli  
> **文档用途**: 后续人员开发参考

---

## 目录

1. [系统环境](#1-系统环境)
2. [硬件配置](#2-硬件配置)
3. [CAN 总线参数](#3-can-总线参数)
4. [电机参数详情](#4-电机参数详情)
5. [ROS2 架构](#5-ros2-架构)
6. [启动流程](#6-启动流程)
7. [接口参考](#7-接口参考)
8. [维护注意事项](#8-维护注意事项)

---

## 1. 系统环境

| 项目 | 值 |
|------|-----|
| 操作系统 | Ubuntu 22.04.5 LTS (Jammy) |
| 内核版本 | 6.8.0-134-generic |
| ROS2 发行版 | Humble |
| CAN 适配器 | PEAK PCAN-USB Pro FD (ID: 0c72:0011) |
| 工作空间路径 | `/home/lionli/ros2_ws` |

### 1.1 PPA 已安装包

| 包名 | 版本 |
|------|------|
| `openarm-can-utils` | 1.2.9-1.ubuntu22.04.1 |
| `libopenarm-can1` | 1.2.9-1.ubuntu22.04.1 |
| `libopenarm-can-dev` | 1.2.9-1.ubuntu22.04.1 |
| `python3-openarm-can` | 1.2.9-1.ubuntu22.04.1 |

### 1.2 本地工作区包

| 包名 | 版本 | 类型 |
|------|------|------|
| `openarm_can` | 1.2.9 | 独立 CMake 库 |
| `openarm_description` | 1.0.0 | URDF/XACRO 模型 |
| `openarm_hardware` | 0.3.0 | ros2_control 硬件接口 |
| `openarm_bringup` | 1.0.0 | Launch + 控制器配置 |
| `openarm` | 1.0.0 | 元包 (metapackage) |
| `openarm_bimanual_moveit_config` | 0.3.0 | MoveIt2 双臂配置 |

### 1.3 环境变量

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 2. 硬件配置

### 2.1 机械臂规格

| 项目 | 值 |
|------|-----|
| 型号 | OpenArm V10 |
| 配置 | 双臂 (right + left) |
| 单臂自由度 | 7-DOF |
| 夹爪 | 1 个 (finger_joint1) |

### 2.2 电机 — CAN ID 映射

| 关节 | 右臂 send_id | 右臂 recv_id | 左臂 send_id | 左臂 recv_id | 电机型号 | 减速比 | Tmax (Nm) | Vmax (rad/s) |
|------|-------------|-------------|-------------|-------------|---------|--------|------------|--------------|
| J1 | 0x01 | 0x11 | 0x01 | 0x11 | DM8009 | 9:1 | 54 | 45 |
| J2 | 0x02 | 0x12 | 0x02 | 0x12 | DM8009 | 9:1 | 54 | 45 |
| J3 | 0x03 | 0x13 | 0x03 | 0x13 | DM4340 | 40:1 | 28 | 10 |
| J4 | 0x04 | 0x14 | 0x04 | 0x14 | DM4340 | 40:1 | 28 | 10 |
| J5 | 0x05 | 0x15 | 0x05 | 0x15 | DM4310 | 10:1 | 10 | 30 |
| J6 | 0x06 | 0x16 | 0x06 | 0x16 | DM4310 | 10:1 | 10 | 30 |
| J7 | 0x07 | 0x17 | 0x07 | 0x17 | DM4310 | 10:1 | 10 | 30 |
| Gripper | 0x08 | 0x18 | 0x08 | 0x18 | DM4310 | 10:1 | 10 | 30 |

> 左右臂 CAN ID 相同，通过不同 CAN 接口区分 (can0=右臂, can1=左臂)

---

## 3. CAN 总线参数

### 3.1 接口配置

| 参数 | can0 (右臂) | can1 (左臂) |
|------|------------|------------|
| 模式 | CAN FD | CAN FD |
| 标称波特率 (仲裁) | 1,000,000 bps | 1,000,000 bps |
| 数据波特率 | 5,000,000 bps | 5,000,000 bps |
| 采样点 (SP) | 0.75 | 0.75 |
| 数据采样点 (DSP) | 0.75 | 0.75 |
| DSJW | 2 | 2 |
| 自动重启 | 100 ms | 100 ms |
| MTU | 72 | 72 |
| 控制模式 | MIT (Code: 1) | MIT (Code: 1) |
| 电机内部波特率 | 5 Mbps (Code: 9) | 5 Mbps (Code: 9) |

### 3.2 CAN FD 帧格式

达妙电机协议使用 8 字节数据帧，CAN FD 帧标志位 `CANFD_BRS`（比特率切换）。

### 3.3 初始化命令

```bash
# 右臂
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
/usr/bin/openarm-can-cli -i can0 can_configure

# 左臂
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up
/usr/bin/openarm-can-cli -i can1 can_configure
```

---

## 4. 电机参数详情

### 4.1 关键寄存器值 (以右臂 J1 为例)

| 参数 | RID | 值 | 说明 |
|------|-----|-----|------|
| Master ID | 7 | 17 | 应答 CAN ID = 0x11 |
| Motor (ESC) ID | 8 | 1 | 发送 CAN ID = 0x01 |
| Control Mode | 10 | 1 (MIT) | MIT 阻抗控制模式 |
| CAN Baudrate | 35 | 9 (5Mbps) | CAN FD 5Mbps |
| Position Limit (PMAX) | 21 | 12.5 rad | 位置限幅 |
| Velocity Limit (VMAX) | 22 | 45/10/30 rad/s | 因电机型号而异 |
| Torque Limit (TMAX) | 23 | 54/28/10 Nm | 因电机型号而异 |
| Gear Ratio | 20 | 9/40/10 | 因电机型号而异 |
| Over-Temp Threshold | 2 | 100°C | 过温保护 |
| Over-Current Threshold | 3 | 0.8 | 过流保护 |
| CAN Timeout | 9 | 0 | 超时禁用 (0=关闭) |

### 4.2 MIT 控制参数 (ros2_control 默认增益)

| 关节 | KP | KD | 备注 |
|------|-----|-----|------|
| J1 | 20.0 | 2.75 | DM8009 |
| J2 | 20.0 | 2.5 | DM8009 |
| J3 | 20.0 | 0.7 | DM4340 |
| J4 | 20.0 | 0.4 | DM4340 |
| J5 | 5.0 | 0.7 | DM4310 |
| J6 | 5.0 | 0.6 | DM4310 |
| J7 | 5.0 | 0.5 | DM4310 |
| Gripper | 5.0 | 0.1 | DM4310 |

> 控制律: τ = kp · (q_des − q) + kd · (dq_des − dq) + τ_ff

### 4.3 零位定义

- **机械臂零位**：竖直自然下垂姿态
- **夹爪零位**：当前闭合位置（手动设定）
- **位置范围**: ±12.5 rad（电机端），输出端需除以减速比

---

## 5. ROS2 架构

### 5.1 数据流

```
ROS2 Controller (100Hz)
    │
    ▼
controller_manager
    │ CommandInterface (pos/vel/effort)
    ▼
openarm_hardware::OpenArm_v10HW (ros2_control SystemInterface 插件)
    │ read()  ──  CAN recv → 解析 CAN FD 帧 → pos/vel/tau states
    │ write() ──  pos/vel/tau cmds → MIT 控制编码 → CAN send
    ▼
libopenarm_can.so (PPA 1.2.9)
    │ OpenArm → ArmComponent / GripperComponent
    │ DMDeviceCollection → DMCANDevice → Motor
    │ CanPacketEncoder / CanPacketDecoder
    ▼
SocketCAN (Linux Kernel)
    │
    ▼
PEAK PCAN-USB Pro FD → 物理 CAN FD 总线 → 达妙电机
```

### 5.2 ROS2 控制器配置

| 控制器 | 类型 | 关节数 |
|--------|------|--------|
| `joint_state_broadcaster` | JointStateBroadcaster | 全部 |
| `right_joint_trajectory_controller` | JointTrajectoryController | 7 |
| `left_joint_trajectory_controller` | JointTrajectoryController | 7 |
| `right_gripper_controller` | GripperActionController | 1 |
| `left_gripper_controller` | GripperActionController | 1 |

- **更新频率**: 100 Hz
- **状态发布频率**: 50 Hz

### 5.3 关节名称

| 右臂 | 左臂 |
|------|------|
| `openarm_right_joint1` ~ `openarm_right_joint7` | `openarm_left_joint1` ~ `openarm_left_joint7` |
| `openarm_right_finger_joint1` (夹爪) | `openarm_left_finger_joint1` (夹爪) |

### 5.4 Action 接口

| Action | 用途 |
|--------|------|
| `/right_joint_trajectory_controller/follow_joint_trajectory` | 右臂轨迹控制 |
| `/left_joint_trajectory_controller/follow_joint_trajectory` | 左臂轨迹控制 |
| `/right_gripper_controller/gripper_cmd` | 右夹爪控制 |
| `/left_gripper_controller/gripper_cmd` | 左夹爪控制 |

### 5.5 夹爪参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 张开位置 | 0.044 m | 关节空间最大值 |
| 闭合位置 | 0.0 m | 关节空间最小值 |
| 电机映射 | 0.044 m ↔ -1.0472 rad (电机端) | 线性变换 |
| 默认 max_effort | 0.5 | per-unit 电流限幅 |

### 5.6 MoveIt2

- **配置包**: `openarm_bimanual_moveit_config`
- **规划组**: `right_arm`, `left_arm`
- **运动学求解**: KDL (默认)
- **规划器**: OMPL

---

## 6. 启动流程

### 6.1 完整启动顺序

```
1. 24V 供电上电
2. 配置 CAN 接口 (can0 + can1, FD 5Mbps)
3. ros2 launch 启动 (择一):
   - 双臂真机控制
   - MoveIt2 拖动规划
4. 可选: 发送轨迹/夹爪指令
5. 结束: ros2 launch 安全停止 (Ctrl+C), 电机自动禁用
```

### 6.2 常用启动命令

```bash
# 双臂真机控制
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    robot_controller:=joint_trajectory_controller

# MoveIt2 拖动末端轨迹规划
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    use_fake_hardware:=false
```

### 6.3 MoveIt2 RViz 操作

1. 启动后 RViz 自动弹出
2. 左侧 **MotionPlanning** 面板 → Planning Group 选 `right_arm` 或 `left_arm`
3. 在 3D 视图中拖拽末端交互球（红/绿/蓝箭头）
4. Planning 标签页 → **Plan** （预览） → **Plan & Execute**（执行）
5. 夹爪: Joints 标签页 → 设置 `finger_joint1` 值

---

## 7. 接口参考

### 7.1 CLI 工具 (PPA)

| 命令 | 用途 |
|------|------|
| `/usr/bin/openarm-can-cli -i <iface> monitor -d <ms>` | 实时电机仪表盘 |
| `/usr/bin/openarm-can-cli -i <iface> discover` | 扫描总线电机 |
| `/usr/bin/openarm-can-cli -i <iface> show_param` | 查看全部电机参数 |
| `/usr/bin/openarm-can-cli -i <iface> enable` | 使能全部电机 |
| `/usr/bin/openarm-can-cli -i <iface> disable` | 禁用全部电机 |
| `/usr/bin/openarm-can-cli -i <iface> set_zero --id <id>` | 单个电机设零位 |
| `/usr/bin/openarm-can-cli -i <iface> can_configure` | 配置 CAN 接口 |
| `/usr/bin/openarm-can-cli -i <iface> change_baud --canid <id> --baudrate <rate> --save` | 修改电机波特率 |
| `/usr/bin/openarm-can-zero-position-calibration` | 右臂自动零位校准 |
| `/usr/bin/openarm-can-zero-position-calibration --canport can1 --arm-side left_arm` | 左臂自动零位校准 |
| `/usr/bin/openarm-can-demo` | 默认 demo (2电机+夹爪) |

### 7.2 原始 CAN 命令

```bash
# 使能电机 #N: cansend <iface> <00N>#FFFFFFFFFFFFFFFC
# 禁用电机 #N: cansend <iface> <00N>#FFFFFFFFFFFFFFFD
# 监控: candump <iface>
```

### 7.3 Python API (PPA)

```python
from openarm_can import OpenArm

openarm = OpenArm("can0", enable_fd=True)
openarm.init_arm_motors(motor_types, send_ids, recv_ids)
openarm.enable_all()
openarm.get_arm().mit_control_all(mit_params)
openarm.disable_all()
```

### 7.4 C++ API (PPA)

头文件路径: `/usr/include/openarm/`

```cpp
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
// 链接: -lopenarm_can
```

---

## 8. 维护注意事项

### 8.1 关键提醒

| 事项 | 说明 |
|------|------|
| **Flash 写入限制** | 电机 Flash 约 10,000 次写入寿命，不要频繁执行 `--save` |
| **断电生效** | 修改电机波特率后必须断电重启 (冷启动) |
| **CAN 2.0 下改波特率** | 修改电机波特率时 CAN 接口必须处于 CAN 2.0 模式 (非 FD)，改完再切回 FD |
| **零位校准** | 不要重复执行，一次校准即可。如需单独调整夹爪用 `set_zero --id 8` |
| **紧急停止** | 急停按钮始终触手可及。软件紧急禁用: 对 0x01~0x08 发送 `FD` 命令 |

### 8.2 已知问题与解决

| 问题 | 原因 | 解决 |
|------|------|------|
| `show_param` 无响应 | CLI 硬编码 FD=true，需切 FD 才能工作 | 电机必须是 5Mbps FD 模式 |
| discover 后接口被恢复 | discover 结束后自动还原接口配置 | 重新 `can_configure` |
| ROS2 launch 失败 | 未 source workspace | `source ~/ros2_ws/install/setup.bash` |
| 电机波特率混合 (4个FD/4个非FD) | 改波特率后未断电 | 断电重启全部生效 |
| 夹爪位置不对 | 零位偏了 | `set_zero --id 8` 手动设定 |

### 8.3 代码库注意

- `openarm_can` CLI 源码硬编码 FD 模式 (`OpenArm(iface, true)`)，本地编译版需配合 CAN FD 使用
- PPA 的 `libopenarm_can.so` (1.2.9) 是推荐使用的稳定版本
- `openarm_hardware` CMakeLists 通过 `find_package(OpenArmCAN REQUIRED)` 查找 PPA 库
- launch 文件默认 `can_fd=true`，与当前 CAN FD 配置一致

### 8.4 调试诊断

```bash
# CAN 接口状态
ip -details link show can0

# 内核日志
dmesg | grep -i can

# CAN 实时流量
candump can0

# ROS2 节点图
rqt_graph
```
