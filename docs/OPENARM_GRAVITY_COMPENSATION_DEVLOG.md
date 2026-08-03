# OpenArm 双臂重力补偿 — 开发日志

> **日期**: 2026-07-25  
> **维护者**: Lionli  
> **前置文档**: [OPENARM_DEVELOPMENT_REFERENCE.md](OPENARM_DEVELOPMENT_REFERENCE.md)

---

## 目录

1. [问题背景](#1-问题背景)
2. [系统当前配置状态](#2-系统当前配置状态)
3. [改动清单](#3-改动清单)
4. [新增功能详解](#4-新增功能详解)
5. [已知问题与排查方向](#5-已知问题与排查方向)
6. [接口与参数速查](#6-接口与参数速查)
7. [调试命令](#7-调试命令)

---

## 1. 问题背景

### 初始现象
大负载电机（J1/J2 DM8009, J3/J4 DM4340）在 MoveIt2 拖动末端轨迹规划后，物理上**达不到目标位置**。
- J1 目标 90°，实际只到 70° 左右（Kp=20 时）

### 排查路径
MoveIt2 配置 → 达妙电机 MIT 控制参数 → 重力补偿缺失 → URDF 轴方向不匹配 → Kp 过低 → 收敛时间不足

### 当前状态
- Kp=200 (↑ 10x), goal_time=2.0s, 18 DOF 双臂 URDF 重力补偿已启用
- **改善**: J1 从 70° → 先到 ~90° 后掉至 ~85°
- **剩余问题**: 轨迹结束后电机"松了"掉一小角度，根因排查中

---

## 2. 系统当前配置状态

### 2.1 编译环境

| 项目 | 值 |
|------|-----|
| ROS2 | Humble |
| C++ 标准 | C++17 (Pinocchio 4.0.0 要求) |
| 新增系统依赖 | `ros-humble-pinocchio` |

### 2.2 新增构建依赖

| 包名 | 文件 | 新增依赖 |
|------|------|---------|
| `openarm_hardware` | `package.xml` | `pinocchio`, `ament_index_cpp` |
| `openarm_hardware` | `CMakeLists.txt` | `pinocchio`, `Eigen3`, `ament_index_cpp` |

### 2.3 MIT 控制参数（当前值）

| 关节 | 电机 | 减速比 | Kp | Kd | 依据 |
|------|------|--------|-----|-----|------|
| J1 | DM8009 | 9:1 | **200** | **5.0** | OpenArm 官方建议 Kp=200 Nm/rad |
| J2 | DM8009 | 9:1 | **200** | **5.0** | 同上 |
| J3 | DM4340 | 40:1 | **100** | **2.5** | 按扭矩比例缩放 |
| J4 | DM4340 | 40:1 | **100** | **2.5** | 同上 |
| J5 | DM4310 | 10:1 | **40** | **1.0** | 同上 |
| J6 | DM4310 | 10:1 | **40** | **1.0** | 同上 |
| J7 | DM4310 | 10:1 | **40** | **1.0** | 同上 |
| Gripper | DM4310 | 10:1 | 5.0 | 0.1 | 不变 |

> 达妙 MIT 模式范围：Kp 0-500, Kd 0-5  
> 编码: 12-bit, Kp→`uint16=(Kp/500)*4095`

### 2.4 控制器参数（当前值）

```yaml
# openarm_v10_bimanual_controllers.yaml
interface_name: position
command_interfaces: [position]       # 纯 MIT 模式，仅位置
state_interfaces: [position]
goal_time: 2.0                        # 轨迹结束后的收敛等待时间
stopped_velocity_tolerance: 0.01
```

### 2.5 MoveIt2 参数（未改动）

```yaml
# joint_limits.yaml (无加速度限制 → TOTG 默认 1 rad/s²)
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

---

## 3. 改动清单

### 3.1 新增文件

无。所有改动限于现有文件。

### 3.2 修改文件

| 文件 | 改动内容 | 改动类型 |
|------|---------|---------|
| `openarm_hardware/package.xml` | +`<depend>pinocchio</depend>` +`<depend>ament_index_cpp</depend>` | 构建依赖 |
| `openarm_hardware/CMakeLists.txt` | +C++17 +Pinocchio/Eigen3/ament_index_cpp +`bimanual:=true` URDF 生成 | 构建系统 |
| `openarm_hardware/include/.../v10_simple_hardware.hpp` | +Pinocchio 头文件 +`GEAR_RATIOS` +`pinocchio_model_/data_` +`gravity_torques_` +`nv_offset_` +`init_pinocchio_model()` +`compute_gravity_torques()` | 接口 |
| `openarm_hardware/src/v10_simple_hardware.cpp` | 构造函数初始化 → `parse_config()` +urdf_path → `on_configure()` +Pinocchio 初始化 → `write()` +重力补偿 τ_ff → 实现 `init_pinocchio_model()` → 实现 `compute_gravity_torques()` | 核心逻辑 |
| `openarm_bringup/config/.../openarm_v10_bimanual_controllers.yaml` | `goal_time: 0.0` → `2.0` | 控制器参数 |

### 3.3 未修改文件

| 文件 | 原因 |
|------|------|
| `*.xacro` (URDF/ros2_control) | 无需改，`ament_index_cpp` 自动发现 URDF |
| `*.launch.py` | 无需改 |
| `moveit_config/*` | 无需改 |

---

## 4. 新增功能详解

### 4.1 Pinocchio 重力补偿

**数据流**：

```
CMake 构建时:
  xacro v10.urdf.xacro bimanual:=true → 18 DOF 双臂 URDF
  → 安装到 share/openarm_hardware/openarm.urdf

运行时 on_configure():
  init_pinocchio_model()
    → ament_index_cpp 自动发现 URDF
    → pinocchio::urdf::buildModel()
    → 根据 arm_prefix 确定 nv_offset (right_→9, left_→0)

运行时 write() (100Hz):
  compute_gravity_torques()
    → q = zeros(18)
    → q[nv_offset+0..nv_offset+6] = pos_states_[0..6]  # 当前臂关节位置
    → q[nv_offset+7] = finger_joint1 (弧度)
    → pinocchio::computeGeneralizedGravity(model, data, q)
    → gravity_torques_[i] = data.g[nv_offset+i] / GEAR_RATIOS[i]
  → MIT 帧: τ_ff = gravity_torques_[i]
```

**双臂 URDF 关节布局**：

```
nv_idx  0..6   7      8       9..15   16     17
       left_J1-7  left_F1 left_F2  right_J1-7 right_F1 right_F2
          ↑ nv_offset=0              ↑ nv_offset=9
```

### 4.2 Kp/Kd 提升

| 之前 | 之后 | 变化 |
|------|------|------|
| Kp = {20,20,20,20,5,5,5,0.5} | Kp = {200,200,100,100,40,40,40,5} | J1/J2 ↑10x |
| Kd = {2.75,2.5,0.7,0.4,0.7,0.6,0.5,0.1} | Kd = {5,5,2.5,2.5,1,1,1,0.1} | J1/J2 ↑2x, Kd=5 触及上限 |

### 4.3 goal_time 收敛等待

```
之前: goal_time=0.0 → 轨迹结束立即报完成 → 电机未收敛
现在: goal_time=2.0 → 最后位置保持 2 秒 → MIT 控制持续收敛
```

---

## 5. 已知问题与排查方向

### 5.1 当前最优先问题：轨迹结束后的"回掉"

**现象**：
1. 轨迹执行中电机准确到达目标 (90°)
2. ~1-2 秒后（goal_time 到期或轨迹结束时）电机"松了"
3. 掉落约 5° 至 ~85°

**待验证假设**：

| 假设 | 验证方式 |
|------|---------|
| A: controller success 后 pos_commands_ 被清零 | `ros2 topic echo /joint_states` 看 position 是否跳变 |
| B: Kp 单位实际为 N/r（非 Nm/rad），有效刚度低 6.28 倍 | 查达妙官方通信协议文档 |
| C: URDF 惯性参数不准确，τ_ff 有偏差 | 对比多种姿态下 monitor 扭矩与实际位置 |

### 5.2 待修复项

| 优先级 | 项目 | 说明 |
|--------|------|------|
| P0 | 排查"回掉"根因 | 见 5.1 |
| P0 | 无摩擦补偿 | J1 虽无重力但有静摩擦，需在 τ_ff 中叠加 |
| P1 | Kp/Kd 参数化 | 当前硬编码在 hpp，改为从 YAML/XACRO 读取 |
| P1 | 加速度限制启用 | `has_acceleration_limits: false` → TOTG 默认 1 rad/s² |
| P2 | 速度缩放放开 | `default_velocity_scaling_factor: 0.1` → 逐步提到 1.0 |
| P2 | KDL → TRAC-IK | 当前 5ms 超时对 7-DOF 可能不稳定 |

### 5.3 不影响当前功能的已知项

| 项目 | 状态 | 说明 |
|------|------|------|
| gripper velocity/torque 映射 | `read()` 中始终为 0 | TODO 注释，暂不影响使用 |
| Octomap DepthImage 插件 | 启动时报错 | 无关，MoveIt2 对应插件未安装 |
| RT FIFO 调度 | `Operation not permitted` | 非 root 用户正常现象 |

---

## 6. 接口与参数速查

### 6.1 MIT 控制帧编码

```cpp
// dm_motor_control.cpp:133-153
uint16_t kp_uint  = double_to_uint(kp, 0, 500, 12);   // 12-bit
uint16_t kd_uint  = double_to_uint(kd, 0, 5, 12);      // 12-bit
uint16_t q_uint   = double_to_uint(q, -12.5, 12.5, 16); // 16-bit
uint16_t dq_uint  = double_to_uint(dq, -vMax, vMax, 12);
uint16_t tau_uint = double_to_uint(tau, -tMax, tMax, 12);

// CAN 帧 8 字节 [qH|qL|dqH|dqL+kpH|kpL|kdH|kdL+tauH|tauL]
```

### 6.2 电机限制参数

```cpp
// DM8009: pMax=12.5, vMax=45,  tMax=54
// DM4340: pMax=12.5, vMax=8,   tMax=28
// DM4310: pMax=12.5, vMax=30,  tMax=10
```

### 6.3 Pinocchio 重力补偿接口

```cpp
// init_pinocchio_model():
//   返回 true 表示加载成功，gravity_compensation_enabled_ = true
//   失败则日志警告，保持原有行为（τ_ff=0）

// compute_gravity_torques():
//   每周期在 write() 中自动调用，填充 gravity_torques_[0..6]
//   单位: 电机端 Nm（已除以 GEAR_RATIOS[i]）
```

### 6.4 控制器参数位置

| 配置文件 | 路径 |
|---------|------|
| 双臂控制器 | `openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml` |
| MoveIt 控制器映射 | `openarm_bimanual_moveit_config/config/moveit_controllers.yaml` |
| MoveIt ros2_control | `openarm_bimanual_moveit_config/config/ros2_controllers.yaml` |
| 关节限位 | `openarm_bimanual_moveit_config/config/joint_limits.yaml` |

### 6.5 硬件参数（XACRO 传入）

| 参数名 | 说明 | 示例值 |
|--------|------|--------|
| `can_interface` | CAN 接口名 | `can0` / `can1` |
| `arm_prefix` | 臂前缀 | `right_` / `left_` |
| `hand` | 是否启用夹爪 | `true` / `false` |
| `can_fd` | CAN-FD 模式 | `true` |
| `robot_description_path` | URDF 路径（可选，会自动发现） | 通常留空 |

---

## 7. 调试命令

### 7.1 构建

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
colcon build --packages-select openarm_hardware openarm_bringup --allow-overriding openarm_hardware
```

### 7.2 启动

```bash
# CAN 配置
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can0 up
/usr/bin/openarm-can-cli -i can0 can_configure
sudo ip link set can1 down && sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can1 up
/usr/bin/openarm-can-cli -i can1 can_configure

# MoveIt2 双臂
source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py \
    right_can_interface:=can0 left_can_interface:=can1 \
    use_fake_hardware:=false
```

### 7.3 运行时验证

```bash
# 确认重力补偿已启用（看启动日志）
# 应出现: "Pinocchio model loaded: 18 DOF, nv_offset=9, gravity compensation ENABLED"

# 监视电机实时数据
/usr/bin/openarm-can-cli -i can0 monitor -d 200

# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces

# CAN 实时流量
candump can0
```

### 7.4 验证重力补偿效果

```bash
# 在 RViz 中: Planning Group → right_arm → 拖动末端小角度 (10-15°)
# → Plan → Plan & Execute

# 观察点:
# 1. 启动日志: 18 DOF, nv_offset=9 确认使用双臂 URDF
# 2. 运动中: monitor 中 J1/J2 torque 有读数
# 3. 到位后: 是否准确，是否回掉
# 4. 无崩溃/失能
```

---

> **更新记录**:
> - 2026-07-25: 初始版本。新增 Pinocchio 重力补偿、Kp 提升、双臂 URDF、goal_time 调整。记录未解决的"回掉"问题。
