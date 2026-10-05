# 在 RC 场地中用 Mid360 建二维导航地图

当前版本针对 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11。
这份文件对应新复制来的 `dev_ws`，不对应外面的旧目录。
源码已静态检查；本机 Windows 没有 ROS/Gazebo，Linux 编译和物理显示尚未验证。

## 发现的问题与改动

1. 原 launch 默认 `use_livox=false`，因此普通启动根本没有雷达；现在默认 true。
2. 原 RViz 的 Livox PointCloud 未启用；现在默认启用，建图另有 mapping.rviz。
3. dev_ws 的 Livox 包引用了缺失的 meshes/mid360.stl，且 CMake 未安装 meshes；已经补齐。
4. 插件写死 Protobuf/Boost 的旧版本库，缺少插件路径导出；改成 CMake 查找和 Gazebo 导出。
5. 原点云依赖 ROS 节点时钟、frame 依赖 Gazebo 传感器名称；改为显式 livox_frame 与 Gazebo SimTime。
6. 无返回射线原来生成 (0,0,0)；PointCloud2 改为 NaN，避免原点假点。
7. 原 Gazebo planar 插件每次把竖直速度归零，可能干扰重力接地；新插件保留竖直速度，
   并在0.5仿真秒没有新指令后停止平面运动。仍保留平面全向速度控制。
8. 雷达射线代码原来修改安装 link 的 ODE 空间；现在只修改射线碰撞体自己的空间。
9. Livox 增加原宏的均匀盒近似惯量并保留固定关节，避免和底盘合并。
10. 原 dev_ws 没有建图节点；新增点云转 LaserScan 与 SLAM Toolbox 的完整启动流程。

模型数值和推导见 `src/27_rc_simulation/robocon2027_gazebo/docs/mapping_model_notes.md`。
原文件备份在 `backup_20261003`，里面有 COLCON_IGNORE，不参与编译。

完整 RM 参考目录在 Windows 上只复制了 .git 和 .vscode，工作区源码缺失；
本次读取其 HEAD=457cae5 的 Git 文件，参考了 `bringup_sim.launch.py`、
`mapper_params_online_async_sim.yaml` 和 `fastlio_mid360_sim.yaml`。
原工程含 FAST-LIO、地面分割、点云转扫描、SLAM Toolbox 和导航。
按你的要求先完成二维地图，这次使用 Gazebo 真值里程计，不启动 FAST-LIO/导航。
地面分割先用车体高度窗替代，局限是只适合当前地面层；多层台面不是一张二维图能完整表达的。

## 1. 复制到虚拟机并安装依赖

把修改后的整个 `dev_ws/src` 复制回虚拟机的 `~/dev_ws/src`。
同时确认没有重复的 robocon2027_gazebo 或 ros2_livox_simulation 包。
每次测试之前关闭旧的 Gazebo/ROS launch，避免两个 server/同名机器人发布者同时运行。

在 Ubuntu 终端执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install build-essential cmake python3-colcon-common-extensions \
  ros-humble-ament-cmake-auto ros-humble-gazebo-dev ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rviz2 ros-humble-rosidl-default-generators \
  ros-humble-tf2-geometry-msgs ros-humble-sensor-msgs-py \
  ros-humble-pointcloud-to-laserscan ros-humble-slam-toolbox \
  ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard \
  libboost-chrono-dev libprotobuf-dev protobuf-compiler
```

仿真仍使用真实的 `livox_ros_driver2/CustomMsg` 类型，但不需要连接真机。
驱动新增 `LIVOX_BUILD_DRIVER=OFF` 选项，只生成该包原来的消息，不编译硬件驱动，
因此这一步不需要 Livox SDK2。原来的全驱动编译默认仍保留。
不要运行驱动仓库的 build.sh，它会清理旧目录且路径按另一种布局编写。

## 2. 编译修改的三个包

下面使用新的 build/install 目录，避免旧缓存继续加载旧插件，无需删除原目录。

```bash
cd ~/dev_ws
colcon build --symlink-install \
  --build-base build_mapping --install-base install_mapping \
  --base-paths src/27_rc_driver src/27_rc_simulation \
  --packages-select livox_ros_driver2 ros2_livox_simulation robocon2027_gazebo \
  --cmake-args -DLIVOX_BUILD_DRIVER=OFF -DBUILD_TESTING=OFF
source ~/dev_ws/install_mapping/setup.bash
ros2 pkg prefix --share robocon2027_gazebo
ros2 pkg prefix ros2_livox_simulation
```

两个结果都应指向 `~/dev_ws/install_mapping`。所有后续终端都 source 此环境。
找不到依赖或编译失败时，保留第一条错误，不要继续启动旧安装中的版本。

## 3. 先确认雷达和接地

```bash
ros2 launch robocon2027_gazebo rc_simulation.launch.py
```

默认启用雷达，RViz Fixed Frame=odom。原模型轮半径0.06 m；出生高度改为0.08 m，
落地后 /odom 中 z 应接近0.06 m。车体底面约在世界 z=0.06，轮底约在世界 z=0。
场地贴色块可能有毫米级厚度；如果整辆车明显陷入，请检查实际 z，而不是仅增大出生高度。

另一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install_mapping/setup.bash
ros2 topic list
ros2 topic hz /livox/lidar/pointcloud
```

频率检查观察后 Ctrl+C。雷达设置10 Hz，但虚拟机实际墙钟频率受实时因子影响。

```bash
ros2 topic echo /livox/lidar/pointcloud --once --field header
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 topic echo /odom --once --field pose
```

点云 header.frame_id 应为 livox_frame，TF 安装平移约 (0.12,0,0.175)。
PointCloud2 类型的话题是 `/livox/lidar/pointcloud`；`/livox/lidar` 是 CustomMsg，
不能拿后者直接配 RViz 的 PointCloud2 显示。

## 4. 启动二维建图

先 Ctrl+C 关闭上面的仿真 launch。这个入口会自行启动 Gazebo 和小车，不要同时开两次：

```bash
ros2 launch robocon2027_gazebo mapping.launch.py
```

流程为：

```text
Mid360 插件 → /livox/lidar/pointcloud → pointcloud_to_laserscan → /scan
                                                                 ↓
Gazebo 控制器 → /odom 和 odom→dummy → base_link              SLAM Toolbox → /map
                                                                  └→ map→odom
```

mapping RViz 的 Fixed Frame=map，显示 Map、Scan、Livox PointCloud、RobotModel、TF。
启动初期 SLAM 尚未收到有效扫描时，map frame 暂时不存在；不要用静态 map→odom 掩盖问题。

快速诊断（墙钟20秒后汇总）：

```bash
ros2 run robocon2027_gazebo check_mapping.py
```

它检查 /clock、点云有效返回、/scan 有效距离、里程计根高度、TF 和地图有效单元。
二维投影在 base_link frame 中高度0.01~0.55 m，避免地面点并覆盖低边栏、台体侧壁。
参数在 `config/pointcloud_to_scan.yaml`；如果扫描点明显稀疏，先检查有效点数与高度窗，
再调整扫描下采样率。Mid360 保留原30000点扫描窗口，每5个候选点取1个，
实际每帧6000射线，量程20 m，降低虚拟机负担。

## 5. 慢速移动并保存地图

在另一个已 source 环境的终端运行键盘控制：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_chassis
```

先按 z 降低速度，按界面提示的 i/j/l 等移动，按 k 停止。
没有新指令0.5仿真秒后控制器会停平面速度；终端失焦时也应明确停车。
先在地面层绕场地慢速移动，观察 /map 扩张；不要直接爬台阶或把多层场地当成同一平面。

保存时保持建图运行，在另一个终端：

```bash
mkdir -p ~/dev_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/dev_ws/maps/rc_ground \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
```

预期生成 `rc_ground.pgm` 与 `rc_ground.yaml`。这是从雷达扫描生成的占据地图，
不是把 Gazebo world 直接导出成图片。未走过且未被扫描覆盖的区域保持未知。

## 6. 点云/地图空白时按数据链定位

| 现象 | 检查 |
|---|---|
| topic list 没有 /livox/lidar/pointcloud | 是否启用 use_livox；Gazebo 日志里的 libros2_livox.so 加载错误；是否 source install_mapping |
| 有 topic 但没有消息 | 仿真是否暂停；CSV 是否成功读取；查看第一个插件错误 |
| 有消息、RViz 仍空白 | 点云话题类型、Fixed Frame、header.frame_id=livox_frame、时间戳与 /clock；显示 QoS Best Effort |
| 点云有数据，/scan 没有效距离 | 车体到雷达 TF；投影高度窗；扫描订阅者存在（转换器按需订阅） |
| /scan 正常但 /map 不出现 | slam_toolbox 是否 active；odom→base_link TF；ros2 lifecycle get /slam_toolbox |
| 小车仍陷地 | /odom 实际 z；是否加载新 librc_planar_move.so；雷达 link 是否保持固定关节；旧 server 是否残留 |

本次没有通过 Windows 实际运行 Gazebo，也没有证据证明原空白只由某一个原因导致。
雷达默认关闭、网格缺失、版本库写死及缺失建图节点是源码中已确认的问题；
地面穿插仍需上述运行数据验证。源文件与安装目录检查不能替代 Linux 编译和仿真测试。
