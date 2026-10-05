# 三维建图坐标和时间约定

消费者为 ROS 2 Humble、Gazebo Classic 11、FAST-LIO、RViz2。
源是现有 simulation_waking_robot.xacro，单位 m/kg/s/rad，右手系 +x前、+y左、+z上。
所有 link、joint、visual、collision、mesh缩放、惯量沿用 mapping_model_notes.md 的记录。
本次不调整车体几何，仅增加 publish_odom_tf 开关，默认 true，三维入口传 false。

| 关系 | 现有 xacro 中的位置和方向 | 三维使用方式 |
|---|---|---|
| dummy → base_link | 固定零变换 | robot_state_publisher 发布 |
| base_link → imu_link | (0.12, 0, 0.125)，rpy=(0,0,0) | robot_state_publisher 发布 |
| base_link → livox_frame | (0.12, 0, 0.175)，rpy=(0,0,0) | robot_state_publisher 发布 |
| imu_link → livox_frame | (0, 0, 0.05)，旋转单位矩阵 | FAST-LIO mapping.extrinsic_T/R |
| lidar_odom → dummy | 由 FAST-LIO IMU 状态乘静态 imu_link→dummy 得到 | 唯一动态定位 TF |

FAST-LIO 的状态 pos/rot 是 IMU 位姿，并非雷达位姿。/lio/odom 的 child_frame_id
为 imu_link；/cloud_registered_body 也是 IMU 坐标。定位 TF 通过实际 URDF 静态
变换计算，避免错扣雷达偏移，避免给 base_link 加第二个父坐标系。
Gazebo 仍发布 /odom 供对比，它不作为三维建图的定位输入，也不发布 odom→dummy。
不使用 Gazebo 真值位姿拼接点云，不添加假的 map→odom 变换。

所有雷达射线在同一个仿真时间求交，CustomMsg offset_time=0。
simulation.instantaneous_scan=true 时，帧起止都使用 header.stamp，IMU积分推进到
这个时刻，不做帧内运动去畸变，也不人为分配0~100ms的点时间。
此开关默认 false，真实雷达仍走原来的逐点去畸变流程。
雷达、IMU、FAST-LIO、RViz使用 /clock。不要在建图中重置 Gazebo 时间；重置后重新启动。

地图坐标原点是 FAST-LIO 初始化时的 IMU，不等于 Gazebo 世界原点。
启动后静止至少5个仿真秒再低速移动，避免把落地或运动过程当作 IMU 初始化。
配置使用0.10米体素，属于初始参数；没有通过实测保证精度。
当前模型是平面速度控制、单时刻射线和近似惯量，不能据此评估真实雷达运动畸变
或底盘越障动力学。FAST-LIO 不提供回环优化，长距离累积漂移仍需评估。

地图显示和PCD保存使用 ikd-Tree 的当前体素地图，立方体边长100米，大于当前场地。
若以后离开该范围，局部地图裁剪会丢弃远处点；这不是全局回环地图。