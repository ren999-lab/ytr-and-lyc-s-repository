# robocon2027_gazebo

ABU ROBOCON 2027《追寻努山塔拉圣物》场地仿真 — Gazebo Classic 11 + ROS2 Humble。

## 快速开始

新增的小车启动入口及修改说明见外层 [README_start.md](../README_start.md)。
启动 `ros2 launch robocon2027_gazebo rc_simulation.launch.py`，默认保留 RM 车体和 IMU、关闭 Livox；
原 `arena.launch.py` 只启动场地。

```bash
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch robocon2027_gazebo arena.launch.py
```

或用现成脚本 `~/robocon2027_start.sh`。

## 文件

| 文件 | 说明 |
|---|---|
| `scripts/gen_arena.py` | **唯一数据源**,全部尺寸以 mm 集中在顶部"布局参数"段(带中文注释/来源标注)。改完 `python3 scripts/gen_arena.py` 重生成 |
| `worlds/robocon2027_arena.world` | 生成的场地世界(SDF,全内联,含物理物体) |
| `docs/arena_topdown.svg` | 俯视标注图,浏览器打开,用来对图纸 |
| `scripts/step2gazebo.py` | (实验)STEP→网格 直转 Gazebo 外观层,见下 |
| `worlds/arena_cad.world` | (转换成功后)CAD 原貌单色层 |

## 坐标系 / 单位

mm;x 东(红方 x<0)、y 北、z 上,地面顶面=0,场地中心=原点。

## 尺寸来源

v2 起,**区域摆放全部按 CAD 总装配实测**
(`/home/ytr/2027RC场地总装配-by风雅荷/2027场地总装配.STEP`),
CAD(x,y↑,z) → 本文件(x,z,y−25):

- 地面:11×11 m,红/蓝各半。启动/重试区 700×700 **在四角北侧**:红(-5150,5150)(-4350,5150),蓝对称。
- 存放区 2000×1000 **在南侧外角**:红中心(-4500,-5000)。
- 天空方块共享区 1200×1200 在**北中**(0,4250);圣物柱+附加共享 1000×1000 在**南中**(0,-4250)。
- 一层台面 6000×6000@600、二层 3000×3000@900 同心;中央柱 Ø270@1700;圣物柱 Ø270@500。
- 建造点位 **一层 6 个**((±2700,±2700)+(0,±2700))、**二层 4 个**((±1250,±1250))。
- 一层重试区 700 在台面 (±1800,-2600);二层台阶贴台体西/东;一层台阶贴外缘。
- 物体按规则 v1.1:大地方块 0.6 kg、天空方块 0.22 kg、圣物 Ø200–210。

### ⚠ 需要你肉眼核对/确认(规则图没给、CAD 也没画)

1. **斜坡**:规则有(3500),CAD 无——位置/是否保留?生成器里 `RAMP_ENABLED`。
2. **交接区 1000×1000**:规则有,CAD 没画——先放在一层台面内侧各自台阶入口(参数 `TRANS_RED/BLUE`)。
3. L1 共享区米黄块(`SHOW_L1_SHARED`)、台阶/挡板的高宽、地面分隔栏细节多为 [推断]。
代码里凡标 `[推断]` 的都是这类,数字旁有注释,直接改即可。

## STEP → Gazebo(实验)

```bash
python3 scripts/step2gazebo.py   # 默认参数
```

该脚本用 gmsh 把 STEP 网格化并换算到 Gazebo 系,产出 `models/cad_arena/meshes/arena.stl`
与 `worlds/arena_cad.world`(单色外观层,默认不含碰撞)。

注意:这套 CAD 里有大量 0.1mm 贴图板和共面结构,gmsh 整场网格化很慢;
脚本已先滤掉贴图/物件,但若仍超时,请把细度参数调大,或日后按子装配单独转。
正常玩法建议直接用 `gen_arena.py` 参数化场地(尺寸已对齐 CAD)。
