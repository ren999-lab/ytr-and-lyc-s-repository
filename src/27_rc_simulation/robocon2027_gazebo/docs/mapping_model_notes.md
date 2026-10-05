# 二维建图模型与坐标系记录

目标：Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11（ODE）、RViz2、SLAM Toolbox。
单位 m/kg/s/rad，车体 +x 前、+y 左、+z 上。
使用原 RM xacro 作为源；场地 world 保留复制版本，没有重新生成。

原车体与轮子、IMU 几何和惯量说明见 `rm_robot_notes.md`（若原复制没有附带该文件，
以下数值为直接从 xacro 读取的记录）：

| link | frame、几何、父 joint | 惯量与质量 |
|---|---|---|
| dummy | frame-only 根，和 base_link 重合 | 无 |
| base_link | dummy_joint 零变换；box 0.2×0.3×0.1，中心 z=0.05，绕 z 旋转 π/2 | 8.2 kg，对角惯量 0.01，原始近似，未重新测量 |
| wheel_1 | base_to_wheel1，(0.1,0.13,0) | 0.5 kg，对角惯量 0.01，原始近似 |
| wheel_2 | base_to_wheel2，(-0.1,0.13,0) | 同上 |
| wheel_3 | base_to_wheel3，(0.1,-0.13,0) | 同上 |
| wheel_4 | base_to_wheel4，(-0.1,-0.13,0) | 同上 |
| imu_link | imu_joint 固定，(0.12,0,0.125)，rpy=0，box 0.05³ | 0.01 kg，对角惯量 1e-6，原始近似 |
| livox_frame | livox_frame_joint 固定，默认 (0.12,0,0.175)，rpy=0 | 新增原宏中的 box_inertia：0.4 kg，0.1×0.06×0.06 均匀盒近似；中心0，惯性 frame rpy=(π/2,0,π/2)，继承原宏 |

四个轮关节 continuous，rpy=(-π/2,0,0)，轴为 joint 的 +z，即车体 +y。
正旋转对应向前滚动。轮半径 0.06 m、宽度0.05 m；质心在轮中心。
visual 和 collision 保持一致。Livox 只有 visual 和传感器，无外壳 collision；
网格文件沿用原 0.0008 缩放与原 visual origin，未凭外观猜测新尺寸。

接地推导：车体底部在 base_link 的 z=0，轮底在 z=-0.06；
默认场地 field_base 的顶面在世界 z=0，因此水平静止时 base_link/dummy 应约在 z=0.06。
出生高度0.08是轮半径再加0.02间隙，让物理引擎落地；不是给车增加固定抬升 TF。
场地视觉贴色块有少量厚度且部分不含 collision，可能出现毫米级视觉重叠；
严重陷入地面需查实际模型 pose、碰撞和是否加载旧插件。

旧 planar 插件每次把 z 速度改为0，会干扰竖直接触。新 `librc_planar_move.so`
保留真实 z 速度及 roll/pitch 角速度，直接控制平面 x/y/yaw，仍不是轮胎驱动力模型；
无指令超过0.5仿真秒后平面速度指令清零。
轮角度仍由 joint_state_publisher 零值提供，不能用它判断实际轮转速。
不据此保证跨台阶、爬坡和真实底盘动力学。

雷达 link 保留固定关节，不与底盘合并；射线 collision 只改自身 ODE 空间，
不更改安装 link 的空间。显式 frame_name=livox_frame，点云 stamp 来自 world SimTime。
所有射线同一仿真时刻求交，因此 CustomMsg offset_time=0，不伪装逐点运动扫描时间。
二维建图用 PointCloud2；后续若用 FAST-LIO，需要重新验证原始点时间模型。

TF 唯一发布关系：

```text
SLAM Toolbox:       map → odom
Gazebo controller:  odom → dummy
robot_state_publisher: dummy → base_link → 各轮/imu_link/livox_frame
```

不启动真实 Livox 驱动节点，不启动 FAST-LIO，也不发布假的 map→odom 静态 TF。
点云转扫描将点转换到 base_link，以高度0.01~0.55 m过滤（水平接地时世界约0.07~0.61 m），
避开地面并保留低边栏和台体侧壁。投影原点选车体 frame，而不是忽略前移0.12 m的雷达偏置。
这些是当前地面层建图的初始参数；二维地图不能描述多层场地的可通行关系。