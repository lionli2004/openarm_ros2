# openarm_description 补丁(相对官方旧版 1.0 结构)

**背景**:官方 enactic/openarm_description 已升级到 2.0(openarmv2.urdf),旧的 urdf/ 目录被官方废弃。
本补丁基于**旧版 1.0 结构**(本地仓库 HEAD af6f035),无法直接 PR 到官方当前 main。

**改动内容**(3 个 xacro,参数透传链):
- `openarm_robot.xacro`:宏参数 + 传递(teaching_mode/teaching_gain_scale/calib_file)
- `v10.urdf.xacro`:顶层 arg 定义 + 传递
- `openarm.bimanual.ros2_control.xacro`:双硬件块 calib_file 参数 + teaching 参数

**应用方式**:在旧版 1.0 结构的 openarm_description 上覆盖这 3 个文件即可。
