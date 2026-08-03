# OpenArm Teaching-Mode Experiments

实验脚本(备份)。运行需要 CAN 硬件与 `PYTHONNOUSERSITE=1 sudo python3 <script>`(绕过 numpy 2.x 与 pinocchio 4.0 冲突)。

**注意**: 这些脚本是开发过程中的 Python 直发工具链, 最终方案已转为 ROS2 硬件层 teaching 模式
(见 docs/OPENARM_TEACHING_MODE_GUIDE.md)。exp1 的结论(τ_ff 输出端单位)已被生产代码采用。
实验数据(CSV)不纳入备份, 可重新生成。
