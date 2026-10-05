# RC 仿真三维建图（FAST-LIO）

文件已放进 `dev_ws/src`。完整 ROS 2 FAST-LIO 在
`src/27_rc_localization/src/FAST_LIO`，含固定版本 ikd-Tree 的源码和头文件。
不要复制 `.reference`、`backup_20261004_3d` 或 Windows 的 `.git` 到构建目录。
可使用 `dev_ws_3d_src.zip`，里面的 `src/` 可合并进虚拟机 `~/dev_ws/`。
复制前备份虚拟机现有 `src`；解压会覆盖同名源码，不覆盖 build/install。

## 1. 安装依赖与编译

在 Ubuntu 22.04 / ROS 2 Humble 的新终端中操作。不要先 source 旧的 install。

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install libeigen3-dev libpcl-dev libomp-dev \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs ros-humble-tf2-msgs \
  ros-humble-std-srvs ros-humble-visualization-msgs ros-humble-sensor-msgs-py
cd ~/dev_ws
colcon build --symlink-install --parallel-workers 1 \
  --build-base build_3d --install-base install_3d \
  --base-paths src/27_rc_driver src/27_rc_simulation src/27_rc_localization \
  --packages-select livox_ros_driver2 ros2_livox_simulation robocon2027_gazebo fast_lio \
  --cmake-args -DLIVOX_BUILD_DRIVER=OFF -DBUILD_TESTING=OFF
```

沿用你已经成功安装的 Gazebo、xacro、robot_state_publisher 等依赖。
`LIVOX_BUILD_DRIVER=OFF` 只关闭硬件SDK驱动，保留 FAST-LIO 使用的真实 Livox 消息类型。
编译 FAST-LIO 可能比原来的三个包耗时更久。内存紧张时先关闭 Gazebo/RViz。
若同一包仍内存不足，可在编译命令前加 `CMAKE_BUILD_PARALLEL_LEVEL=1`。
多行命令的反斜杠必须是行末最后一个字符，后面不能有空格。

## 2. 启动

先关闭之前的仿真、二维建图、RViz和键盘节点，以免同名话题或TF冲突。

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install_3d/setup.bash
ros2 launch robocon2027_gazebo mapping_3d.launch.py
```

同时启动 Gazebo、小车、雷达、IMU、FAST-LIO 和 RViz。
FAST-LIO 等3个仿真秒后启动，启动后继续静止至少5个仿真秒，等待
`IMU Initial Done` 和地图点云出现，再移动。仿真速度很慢时，真实等待时间要更长。
RViz 固定坐标系 `lidar_odom`，`3D Voxel Map` 显示 `/Laser_map`，
`Registered Scan` 显示当前配准点云，`LIO Path` 显示轨迹。
初始化前还没有 `lidar_odom` TF，RViz短暂报坐标系不存在属于等待初始化。

新终端：

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install_3d/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/cmd_vel_chassis -p speed:=0.2 -p turn:=0.5 \
  -p repeat_rate:=10.0 -p key_timeout:=0.6
```

键盘窗口保持焦点：`i`前进，`j/l`原地转向，`u/o`边走边转，`k`停止。
以低速绕场地，经过有多个方向墙面和台体的区域；避免高速急转和碰撞。

## 3. 保存三维地图

小车停稳后，新终端加载同样环境，再执行：

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
ls -lh ~/dev_ws/maps/rc_3d.pcd
```

服务必须返回 `success: true`；空地图、目录权限或写盘失败会返回原因。
保存当前融合的体素地图，而非只保存最后一帧。正常 Ctrl-C 退出也会尝试保存，
但建议退出前手动保存一次，避免进程被强制结束。
再次保存会覆盖同名文件；另一次实验可以在启动时指定新路径：

```bash
ros2 launch robocon2027_gazebo mapping_3d.launch.py map_path:=$HOME/dev_ws/maps/rc_3d_run2.pcd
```

`.pcd` 是三维点云，用 CloudCompare/PCL等工具查看；它不能直接替代 Nav2 的 `.pgm/.yaml`。
三维建图本身也没有包含导航、回环优化和多层场地的路径规划。

## 4. 检查与精度验证

停止小车后：

```bash
ros2 run robocon2027_gazebo check_mapping_3d.py
```

该命令只读检查点云、IMU、仿真时间、配准点云、地图、TF和保存服务。
PASS说明数据链连通，不说明地图精度达标。
如果没有输出点云，先看 FAST-LIO 终端的第一条错误，再检查：
