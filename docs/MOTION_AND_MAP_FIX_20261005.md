# Gazebo 直行偏航与三维地图重影检查

这次以本仓库和用户随后复制的虚拟机 FAST-LIO 源码为准。Windows 没有 ROS 2/Gazebo，
以下是代码检查及针对性修复；还没有完成虚拟机运行验证。

## 已确认的问题与修改

1. `src/27_rc_simulation/robocon2027_gazebo/src/rc_planar_move.cpp` 原来调用
   `Model::SetLinearVel/SetAngularVel`。Gazebo Classic 会对模型中所有 link 设置速度，
   包括四个 continuous 轮关节的子 link；设车身 yaw 速度时，也会抹掉轮子的滚动角速度。
   现在只对 canonical 车身 link 设置速度，轮子继续由接触与关节运动决定滚动。
   此外将 xacro 中 `update_rate` 改为 0：每个物理步施加指令，而非每 10 个物理步施加一次。
   仍保留重力方向速度及 roll/pitch 角速度；不锁住位置或航向，不修改真实 odom 输出。
   这仍是直接速度控制模型，不能作为实际麦克纳姆轮驱动和越障动力学模型。

2. 雷达插件原来只在 `UpdateRays()` 的碰撞检测阶段加物理锁。
   Gazebo `MultiRayShape::Update()` 在此前逐条调用 `ODERayShape::Update()`，
   每条射线读取安装 link 的 `WorldPose()`。约 6000 条射线计算期间，车身可能移动，
   与 `offset_time=0` 的整帧瞬时扫描约定不符。
   现在在外层锁住方向更新、整帧射线更新和扫描时间捕获；同一帧不会跨越多个 link 姿态。
   序列化和 ROS 发布在锁外。已有内层锁是递归锁，可以重复取得。

3. 原始配置雷达最大量程为 200 m，FAST-LIO 的 `mapping.det_range` 为 20 m。
   `det_range` 不会自动替你剔除所有输入远点。
   提供的 `maps/rc_3d.pcd` 有 430130 点，约 70.0% 距离建图原点超过 20 m，32.6% 超过 50 m。
   这些距离以地图原点计算，不等于采样时到雷达的距离，但说明地图有大量远处点。
   world 中兜底地面使用 plane，ODE plane 碰撞实际上无限延伸。
   将仿真雷达量程统一到 20 m，减少无关远处地面。量程调整本身不能保证消除偏航。

4. FAST-LIO 原初始化只要求少量 IMU 样本，不检查是否仍在落地或转动。
   虚拟机提供的 `Log/mat_pre.txt` 第一条记录中 gyro bias 为约
   `[-0.0397292, 0.000423635, 0.000197816] rad/s`；这说明初始均值并非静止零角速度，
   支持排查落地运动被当作偏置，但没有真值日志，不能由此独断全部地图误差来源。
   现在仿真模式等待观测到稳定 IMU 持续 1 个仿真秒：角速度模长 <=0.02 rad/s、
   加速度模长与重力相差 <=0.5 m/s²，拒绝非有限值；发现新运动后重新等待。
   稳定检查通过之后才收集初始化均值，打印实际初始化偏置。
   参数 `simulation.initialization_stationary_seconds` 默认为 1.0；仅在
   `simulation.instantaneous_scan=true` 时启用，真实雷达去畸变路径不变。

参考实际 Gazebo 实现：
- https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/Model.cc
- https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/MultiRayShape.cc
- https://github.com/gazebosim/gazebo/blob/gazebo11/gazebo/physics/ode/ODERayShape.cc

## Git 同步问题

仓库原只用 gitlink 记录 FAST-LIO 上游提交 `2fffc570a25d0df172720bac034fbdb6a13d2162`。
虚拟机未提交的修改不会跟随主仓库推送，主机初始检出只有空目录。
现已将用户本次复制的 FAST-LIO（包括 ikd-Tree）改为普通源文件纳入主仓库，保留许可证。
没有用上游版本或旧主机副本覆盖用户源码。运行日志和 PCD 输出不纳入 FAST-LIO 源文件提交。
另外 Livox 的 `package.xml` 原来被忽略，干净检出无法识别该包；已加入 ROS 2 清单。
Point-LIO 仍为原有 gitlink，且缺少 `.gitmodules` 映射；本次 FAST-LIO 建图不使用它，
不要在这个工作流运行 `git submodule update --init --recursive`。

本次修改需要提交并推送后，虚拟机才能拉取。本地源码不会通过拉取自动变成新的 `.so`。
若虚拟机 FAST-LIO 目录仍是独立 Git 仓库，切换为普通目录时可能遇到现有文件阻止更新；
先保留该目录的备份，或在新目录检出修复后的主仓库再编译。

## 虚拟机验证

停止旧的 launch。在一个新终端中进入包含本仓库 `src` 的根目录，先 source Humble。
用新的构建和安装目录避开仓库带来的旧 CMake 缓存和旧二进制：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 1 --build-base build_motion_fix --install-base install_motion_fix --base-paths src/27_rc_driver src/27_rc_simulation src/27_rc_localization --packages-select livox_ros_driver2 ros2_livox_simulation robocon2027_gazebo fast_lio --cmake-args -DLIVOX_BUILD_DRIVER=OFF -DBUILD_TESTING=OFF
source install_motion_fix/setup.bash
ros2 launch robocon2027_gazebo mapping_3d.launch.py map_path:=$HOME/dev_ws/maps/rc_3d_motion_fix.pcd
```

确认底盘启动输出 `Controlling chassis link [...]`，等 `IMU Initial Done` 和新的偏置输出，
随后在安全空地低速直行、停下，再缓慢转弯。旧地图保留，新地图使用新名字。
诊断脚本在另一个已 source 同一安装目录的终端运行，工作目录仍为仓库根目录：

```bash
python3 tools/diagnose_motion.py --duration 60 --output motion_report.json
```

脚本只订阅数据，不发布运动指令。不需要安装进 ROS 包，也不依赖额外的点云软件。
它比较不同初始坐标原点的相对 yaw，避免把出生时 -90° 与 FAST-LIO 初始 0° 的正常差异当作误差。

- `gazebo_during_straight_commands`：直行指令有效期间 Gazebo 实际航向变化。
  如果这里明显变化，继续查碰撞、底盘模型和指令，不能先归因于 FAST-LIO。
- `lio_vs_gazebo`：按时间对齐并扣除初始角度差后的估计误差。
  如果 Gazebo 直行稳定但这个误差增长，继续查 LIO 配准及传感器数据。
- `imu_z_vs_gazebo_level_chassis_only`：水平车身时，IMU z 角速度积分与真值 yaw 的比较。
  存在明显 roll/pitch 或 IMU 时间缺口时，不能用此近似直接判断 yaw 精度。
- `frames` 和 `dummy_dynamic_parents`：三维建图时根的动态父坐标应只有 `lidar_odom`。

没有直行片段、数据缺失或配对数量不足，就不能判定修复是否有效。观察到车身转向本身
也不足以解释地图重影：定位若正确跟踪实际转向，地图应仍能重合。

## 本地主机检查

### 后续运行日志：一直出现 No point

虚拟机的四个包已编译成功。后续启动输出连续 342 次 `No point, skip this scan!`，
未出现 `IMU Initial Done` 或 `Initialize the map kdtree`。
原提示将“静止检查尚未通过、Process 主动保留空输出”误报成“没有雷达点”，
但同步回调已经取得可预处理点云和 IMU，不能仅凭这条提示判断雷达没发点云。

后续修正将初始化等待单独打印（每 3 个仿真秒最多一次），包含等待原因、
已持续稳定秒数、批次最大角速度模长、批次最大加速度模长偏差。
保留原检查阈值，不在没有实测数据时直接取消或放宽。
同时让 `diagnose_motion.py` 输出 IMU 统计和连续稳定时间；诊断初始化时全程保持静止。
若稳定状态一直达不到要求，要根据这些数值排查车身振动、传感器数据与采样频率。
这次信息不足以断言是哪个阈值导致等待。

FAST-LIO 的 `Log` 目录原来只含被忽略的运行日志，Git 不保存空目录，所以干净检出缺目录。
现在增加占位文件，调试日志打开失败也会提示准确路径，不再声称整个源码目录不存在。
调试日志缺失不是初始化阻塞的原因。

已运行编辑后 C++ 函数的回归测试（车身控制、静止初始化、整帧射线锁）、诊断角度对齐测试、
FAST-LIO 瞬时/真实扫描同步边界测试，以及实际 xacro 展开三种配置的检查。
这些检查没有运行 Gazebo、没有验证整包 C++ 编译，也没有测量修复后的建图精度。

```bash
python tests/test_motion_fixes.py
python tests/test_fastlio_sync.py
```
