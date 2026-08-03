# OpenArm V10 示教模式(拖拽示教)开发指南

> **生成日期**: 2026-08-03
> **维护者**: Lionli
> **前置文档**: [OPENARM_DEVELOPMENT_REFERENCE.md](OPENARM_DEVELOPMENT_REFERENCE.md)、[OPENARM_GRAVITY_COMPENSATION_DEVLOG.md](OPENARM_GRAVITY_COMPENSATION_DEVLOG.md)
> **文档用途**: 记录示教模式(柔顺拖动 + 任意位置悬停 + 拖拽示教演示)的完整开发过程、核心结论与使用指南

---

## 目录

1. [背景与目标](#1-背景与目标)
2. [核心结论速查](#2-核心结论速查)
3. [系统架构与数据流](#3-系统架构与数据流)
4. [代码改动清单](#4-代码改动清单)
5. [使用指南](#5-使用指南)
6. [参数速查表](#6-参数速查表)
7. [开发过程关键发现](#7-开发过程关键发现)
8. [已知问题与排查方向](#8-已知问题与排查方向)
9. [调试命令](#9-调试命令)
10. [后续工作](#10-后续工作)

---

## 1. 背景与目标

### 1.1 初始问题

重力补偿工作(2026-07-25)后遗留两个问题:
- **轨迹结束后"回掉"约 5°**:Kp=200 高增益下,轨迹执行完成后电机松弛掉角度
- **无法柔顺拖动**:高增益位置环使手臂僵硬,不能直接示教

### 1.2 目标

实现**示教模式**(teaching mode):
- **柔顺拖动**:低增益 + 位置目标跟踪,手臂可徒手拖拽
- **任意位置悬停**:松手后靠重力补偿 + 关节摩擦停在原地
- **拖拽示教演示**:记录拖动轨迹 → 自动回放展示(带 RViz 面板前端)

---

## 2. 核心结论速查

| # | 结论 | 影响 |
|---|------|------|
| 1 | **达妙 MIT 帧的 q/dq/tau 全部是输出端(关节端)单位** | 重力补偿**不得**除以减速比 |
| 2 | **`÷GEAR_RATIOS` 是"回掉"的根因** | 重力补偿被削弱 9-40 倍(实验数据:g/9 与 τ_ff=0 漂移几乎相同) |
| 3 | τ_ff = +g(输出端)使 J2 在 45° 悬停仅 -0.2°/5s | 补偿精度 <0.1 Nm,回掉问题随根因修复消失 |
| 4 | 拖动示教 = 低增益(Kp×0.1)+ q_des 跟踪反馈 + τ_ff 重力补偿 | 100Hz 生产环境下稳定工作 |
| 5 | 限位检查需 0.01 rad 死区 | J4 零位=下界(0.0),读数噪声 -1e-4 即误触发 |
| 6 | Python 直发 CAN 无法稳定控制(更新率不足) | 一切控制走 ROS2 ros2_control 100Hz 路径 |

---

## 3. 系统架构与数据流

### 3.1 teaching 模式的本质

```
τ_cmd = Kp·scale·(q_des − q) + Kd·scale·(0 − q̇) + τ_ff
```

| 项 | 拖动模式(teaching) | 回放模式(normal) |
|---|---|---|
| gain scale | 0.1(有效 Kp 20/10/4) | 1.0(Kp 200/100/40) |
| q_des | 实时跟踪 pos_states_(反馈) | 轨迹控制器指令 |
| dq_des | 0 | 控制器指令 |
| τ_ff | Pinocchio 重力补偿(输出端 Nm) | 同左 |

- 拖动:位置环"不知道"自己被拖走 → 无阻力
- 松手:τ_ff 抵消重力,静摩擦 > 模型误差 → 悬停

### 3.2 运行时模式切换链路

```
RViz 面板 (openarm_demo/DemoPanel)
    │ service (/openarm_demo_controller/set_mode)
    ▼
demo_controller 节点
    │ Float64MultiArray → /left_teaching_switch/commands (×2)
    ▼
forward_command_controller (left/right_teaching_switch)
    │ finger_joint1 的 effort 命令接口
    ▼
硬件层 write() 读取 tau_commands_[7]:
    > 0.5  → teaching (拖动)
    < -0.5 → 紧急 disable (latch,on_configure 清除)
    其他   → 回落启动参数 teaching_mode_
```

### 3.3 演示功能数据流

```
记录:  demo_controller 订阅 /joint_states (50Hz) → CSV [t, 16关节]
回放:  demo_controller 读 CSV → FollowJointTrajectory action → 左右臂轨迹控制器
```

---

## 4. 代码改动清单

### 4.1 硬件层 `openarm_hardware`(核心)

| 文件 | 改动 | 说明 |
|------|------|------|
| `src/v10_simple_hardware.cpp` | **删除 `÷GEAR_RATIOS`**(根因修复) | τ_ff 直接用 Pinocchio 输出端重力矩 |
| 同上 | teaching 分支(write()) | q_des←pos_states_, gain scale, dq_des=0 |
| 同上 | 运行时开关读取 | finger effort 命令接口:>0.5 teaching, <-0.5 estop |
| 同上 | teaching 限位保护(URDF 解析 + 0.01 rad 死区) | 拖过 URDF 限位 → disable |
| 同上 | on_activate 跳过回零(teaching 模式) | 臂保持当前姿态 |
| 同上 | on_configure 清除 estop latch | 恢复路径 |
| 同上 | 插值式 return_to_zero(官方移植) | 200 步 × 10ms |
| 同上 | on_deactivate 3 次循环 disable(官方) | 可靠失能 |
| 同上 | recv_all(1000→100)(官方) | 缩短阻塞 |
| `include/.../v10_simple_hardware.hpp` | 增益成员化 kp_/kd_(可参数覆盖 kp1..kp7) | 官方格式 |
| 同上 | teaching_mode_/teaching_gain_scale_/estop_triggered_/限位成员 | 新增 |
| `CMakeLists.txt` | +urdf 依赖 | URDF 限位解析 |

### 4.2 参数链路

| 文件 | 改动 |
|------|------|
| `openarm_description/urdf/ros2_control/openarm.bimanual.ros2_control.xacro` | +teaching_mode/teaching_gain_scale 参数透传(双硬件块) |
| `openarm_description/urdf/robot/openarm_robot.xacro`、`v10.urdf.xacro` | 宏参数 + arg 传递 |
| `openarm_bringup/launch/openarm.bimanual.launch.py` | +teaching 参数、+demo_controller 节点、+switch 控制器 spawn |
| `openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml` | +left/right_teaching_switch(forward_command_controller) |
| `openarm_bringup/rviz/bimanual.rviz` | +DemoPanel 面板定义 |
| `openarm_bringup/package.xml` | +exec_depend |

### 4.3 新增包 `openarm_demo`

| 文件 | 内容 |
|------|------|
| `src/demo_controller.py` | 记录/回放/模式切换/急停/状态(7 个 service) |
| `include/openarm_demo/demo_panel.hpp` + `src/demo_panel.cpp` | RViz 面板插件(Qt) |
| `srv/Replay.srv`、`srv/Status.srv` | 自定义接口 |
| `openarm_demo_panel_plugin.xml` | 插件描述 |

### 4.4 实验脚本(工具,非生产)

`experiments/teaching/`:`common.py`(Arm/GravityModel/Safety)、`exp1_units.py`、`exp2_friction.py`、`exp3_hover.py`、`monitor_joint_states.py`。运行方式 `PYTHONNOUSERSITE=1 sudo python3 <script>`(绕过用户目录 numpy 2.x 与 pinocchio 4.0 的冲突)。

---

## 5. 使用指南

### 5.1 启动(单条命令)

```bash
# CAN 配置(断电重启后才需要)
sudo ip link set can0 down && sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on && sudo ip link set can0 up
/usr/bin/openarm-can-cli -i can0 can_configure
# can1 同理

# 启动:双臂 + 控制器 + RViz(含面板)+ demo controller
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch openarm_bringup openarm.bimanual.launch.py teaching_mode:=false
```

> **必须 `teaching_mode:=false` 用于演示**:模式由面板控制。若 `:=true`,硬件层 sw=0 时回落参数 → 永远 teaching,回放被忽略(demo controller 启动时会打印此警告)。

### 5.2 面板操作(RViz → Panels → OpenArm Drag-Teaching Demo,launch 已预置)

| 步骤 | 按钮 | 现象 |
|------|------|------|
| 1 | 拖动模式 | 臂变软,状态栏 teaching=True |
| 2 | 开始记录 | recording=True |
| 3 | 拖动双臂画轨迹 | — |
| 4 | 停止记录 | saved N samples → csv |
| 5 | 回放模式 | **臂变硬(Kp=200)= 切换成功证据** |
| 6 | 速度选择 → ▶ 回放 | 双臂重演 |
| 7 | ■ 停止回放 | 中途停止 |

### 5.3 注意事项

- **estop 红按钮是 latch 保护**:触发后必须重启 launch 恢复
- 回放是刚性位置控制,回放时人员保持距离
- 轨迹文件:`~/ros2_ws/experiments/teaching/data/demo/`,可重复回放
- 记录/回放包含 16 关节(双臂 7+7+2 夹爪),夹爪动作自然支持

---

## 6. 参数速查表

### 6.1 增益(输出端单位)

| 关节 | 电机 | 减速比 | 正常 Kp/Kd | teaching Kp/Kd (scale 0.1) |
|------|------|--------|-----------|---------------------------|
| J1/J2 | DM8009 | 9:1 | 200 / 5.0 | 20 / 0.5 |
| J3/J4 | DM4340 | 40:1 | 100 / 2.5 | 10 / 0.25 |
| J5-J7 | DM4310 | 10:1 | 40 / 1.0 | 4 / 0.1 |
| Gripper | DM4310 | 10:1 | 5.0 / 0.1 | 不变 |

可通过硬件参数 `kp1..kp7`/`kd1..kd7`/`kp_hand`/`kd_hand` 覆盖(官方格式)。

### 6.2 限位保护

| 项 | 值 |
|----|-----|
| 来源 | URDF(与 RViz 一致,含双臂 offset) |
| 死区 | 0.01 rad(读数量化噪声吸收) |
| 触发 | q < lower−0.01 或 q > upper+0.01 → disable 电机 |
| 关键关节 | 右臂 J2 下限 -0.175 rad(-10°);J4 [0, 2.443](零位=下界);左臂 J2 上限 0.175 rad |

### 6.3 运行时开关(finger effort 命令接口)

| 值 | 行为 |
|----|------|
| > 0.5 | teaching(拖动) |
| < -0.5 | 紧急 disable(latch,on_configure 清除) |
| 其他 | 回落启动参数 teaching_mode_ |

### 6.4 关键事实(实验验证)

- MIT τ 帧单位:输出端 Nm(τ_ff 不除减速比)
- J2=45° 补偿后悬停漂移:-0.2°/5s(τ_ff=0 时 -40.4°,g/9 时 -41.2°)
- 拖动记录采样:50Hz(joint_state_broadcaster),回放点率即采样率

---

## 7. 开发过程关键发现

### 7.1 回掉根因的确认路径(实验 1 Phase B)

Kp=0 手扶 J2=45°,比较 5 种 τ_ff 假设的 5s 漂移:

| τ_ff 假设 | 漂移 | 结论 |
|-----------|------|------|
| 0 | -40.4° | 无补偿即下滑 |
| **+g (6.93 Nm)** | **-0.2°** | **输出端单位正确,符号正确** |
| -g | -45.8°(撞限位) | 符号反会加速坠落 |
| +g/N (÷9) | -41.2° | 除减速比 = 无补偿 |

### 7.2 开发中的坑(按时间)

| 坑 | 现象 | 解决 |
|----|------|------|
| sudo 环境丢失 PYTHONPATH | pinocchio 找不到 | 脚本内 sys.path 注入 + 移除用户 site-packages |
| 交互等待阻塞 CAN | watchdog 300ms 误触发 | select 非阻塞 + keepalive 保活帧 |
| Python 直发更新率低(~20Hz) | 所有增益下极限环振荡 | 放弃 Python 直发,走 ROS2 100Hz |
| J5-J7 用 Kp=200 | 末端小电机严重振荡(±3.8 rad/s) | 逐关节增益(J5-7 用 40) |
| Kp=20 无补偿保持 J2 | 1 rad/s 下滑(回掉现场) | 实验用 Kp=100+ 保持;生产有 τ_ff |
| 限位 margin 0.05 | J4 零位(=下界)立即误触发 | margin→0,再改为死区 0.01 |
| 限位 margin 0 | 读数噪声 -1e-4 误触发 | 死区 0.01 rad |
| 插件 xml `lib/` 前缀 | pluginlib 双 lib 找不到库 | 去掉前缀(与官方一致) |
| 面板库无 RPATH | typesupport 加载失败 | INSTALL_RPATH 指向 prefix/lib |
| 面板节点未 spin | service 请求发出但状态不更新 | 面板自建 executor 线程 |
| 旧 demo_controller 残留 | 同名节点冲突,切换无效 | 唯一实例(launch 一体化后消除) |
| `teaching_mode:=true` + 面板 | 切回放模式无效(参数回落) | 演示必须 `:=false` 启动 |
| can1 未配置/discover 改波特率 | 左臂静默失能 / RX 无信号 | can_configure + discover 后恢复 |

### 7.3 悬停物理判据

```
悬停 = 静摩擦 > |重力模型误差|
```

- 重力模型误差:官方 gravcomp 分支用 1.2 系数修正(~20%),本系统实测补偿后 J2 漂移 0.2°(<0.1 Nm)
- 悬停质量 = 重力补偿精度,与"回掉"同根

---

## 8. 已知问题与排查方向

| 优先级 | 问题 | 说明/方向 |
|--------|------|-----------|
| P1 | J3/J4(40:1)拖动阻力大 | 齿轮箱反向摩擦主导,可接受悬停-only;摩擦前馈(路径 C)可改善 |
| P1 | 无摩擦补偿 | τ_ff 加库仑/黏滞项可提升顺滑度与悬停精度 |
| P2 | 左臂符号未单独验证 | 左右臂镜像,URDF 仅 J7 轴反射;悬停测试未发现异常,但建议专项验证 |
| P2 | Kp 单位(N/r vs Nm/rad)未最终确认 | 实验 1 数据可顺带检验;当前行为已验证可用 |
| P3 | demo controller 退出竞态 | rclpy.shutdown 已 try/except(日志 RCLError 无害) |
| P3 | RViz 惯量警告(finger links) | 已知无关项 |

---

## 9. 调试命令

```bash
# 状态监视(带关节名)
python3 ~/ros2_ws/experiments/teaching/monitor_joint_states.py

# 控制器状态
ros2 control list_controllers    # 需 apt install ros-humble-ros2-control

# 模式开关话题(验证切换链路)
ros2 topic echo /left_teaching_switch/commands
ros2 topic info /left_teaching_switch/commands

# demo 控制器服务
ros2 service list | grep openarm_demo
ros2 service call /openarm_demo_controller/status openarm_demo/srv/Status "{}"

# 构建(改动后)
colcon build --packages-select openarm_hardware openarm_bringup openarm_demo openarm_description --allow-overriding openarm_hardware
```

---

## 10. 后续工作

| 方向 | 内容 | 状态 |
|------|------|------|
| 路径 B | 重力补偿移出硬件层,独立 gravcomp controller + effort 直通(官方 main 架构) | 规划中 |
| 路径 C | 摩擦前馈(Stribeck/库仑+黏滞)辨识与补偿 | 规划中 |
| 参数化 | Kp/Kd 从 YAML/XACRO 读取(官方 main 已有,移植) | 部分完成 |
| 动态模式切换优化 | 切换时自动停回放(已做);estop 恢复路径(已做) | 完成 |
| 左臂符号专项验证 | 左臂悬停多姿态测试 | 待做 |

---

> **更新记录**:
> - 2026-08-03:初始版本。记录示教模式全流程:单位根因修复、teaching 模式、演示功能(记录/回放 + RViz 面板)、限位保护、开发坑与参数。
