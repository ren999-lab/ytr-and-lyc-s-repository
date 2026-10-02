#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ABU ROBOCON 2027《追寻努山塔拉圣物》场地生成器  v2
======================================================================
本脚本是场地的"唯一数据源":改动下面任意一个常量 → 保存 → 重新运行
    python3 gen_arena.py
就会同时重新生成:
  * ../worlds/robocon2027_arena.world   (Gazebo Classic SDF, 全部内联,无外部依赖)
  * ../docs/arena_topdown.svg           (带标注俯视平面图, 用于和图纸核对)

单位:全部为 毫米(mm)。写 SDF 时自动转成米。

坐标系(和 Gazebo 一致):
    x  → 东(East), 红方在 x<0(西), 蓝方在 x>0(东)
    y  → 北(North), 场地范围 [-5500, +5500]
    z  → 向上,      地面(地板块顶面)= 0
    世界原点 = 场地中心

尺寸来源说明(重要):
  [CAD] = 直接量自 CAD 源装配
          /home/ytr/2027RC场地总装配-by风雅荷/2027场地总装配.STEP
          坐标已换算:CAD(x,y,z) → 本文件(x, z, y-25)。CAD 地板块顶面在 y=25。
  [规则] = 规则书(官方 V1.1 / 中文译本)明确数值。
  [推断] = 规则图未给、CAD 也未覆盖,是我按合理方式放置,请对照图纸肉眼确认。
  修改点:凡是"布局"类的都在下面"======== 布局参数 ========"大段里,
  数字旁通常标有来源注释,方便你直接改。
"""

import math, os, html

HERE       = os.path.dirname(os.path.abspath(__file__))
OUT_WORLD  = os.path.normpath(os.path.join(HERE, "..", "worlds", "robocon2027_arena.world"))
OUT_SVG    = os.path.normpath(os.path.join(HERE, "..", "docs",   "arena_topdown.svg"))

# ==========================================================================
# 一、颜色(规则书 §14 颜色表)
#    命名: C_RED 代表队主色/红面; G_xxx 为地面/台面底色; 其余按部件。
# ==========================================================================
C_RED    = (223,  34,  34)   # 红队漆 / 红面 / 红色启动-重试区
C_BLUE   = ( 50,   0, 255)   # 蓝队漆 / 蓝面 / 蓝色启动-重试区
G_RED    = (240, 210, 210)   # 地面(红方)底色
G_BLUE   = (170, 210, 230)   # 地面(蓝方)底色
SHARED   = (245, 240, 200)   # 地面共享区 / L1 共享区 米黄
L1_RED   = (235, 180, 160)   # 一层(红方台面)
L1_BLUE  = (150, 215, 220)   # 一层(蓝方台面)
STAIR_R  = (235, 180, 160)   # 一层台阶(红) [规则] 同 L1 台面色
STAIR_B  = (150, 215, 220)   # 一层台阶(蓝)
L2_GREY  = (190, 190, 185)   # 二层台面 / 挡板 / 侧墙
RAMP_R   = (200, 150, 140)   # 斜坡(红)
RAMP_B   = (130, 180, 200)   # 斜坡(蓝)
DIV      = (100,  62,   0)   # 分隔栏 / 场地边界 木棕色
SPOT_C   = ( 40, 100,  50)   # 建造点位 绿
TR_R     = (245, 170,  60)   # 交接区(红)
TR_B     = ( 60, 170, 245)   # 交接区(蓝)
WOOD     = (130, 100,  72)   # 台体侧面/结构的木质色(纯外观)
BASE     = ( 92,  70,  48)   # 最底下的木地板块
GOLD     = (218, 165,  32)   # 圣物(灵石)
EARTH_R  = C_RED             # 大地方块(红方)
EARTH_B  = C_BLUE            # 大地方块(蓝方)

# ==========================================================================
# 二、规则固定尺寸(mm)  —— [规则],一般不用改
# ==========================================================================
FIELD    = 11000.0                    # 外场 11m × 11m
HLF      = FIELD / 2                  # 半场 5500
L1       = 6000.0                     # 一层台面边长
L1_H     = 600.0                      # 一层高于地面
L2       = 3000.0                     # 二层台面边长
L2_H     = 900.0                      # 二层高于地面(= 一层600 + 300)
STEP_DEP = 300.0                      # 每级台阶 "长"(进深)
STEP_W   = 1000.0                     # 每级台阶 "宽"
STEP_H   = 150.0                      # 每级台阶高
START_S  = 700.0                      # 启动区/地面重试区 700×700 [规则]
STOR_L   = 2000.0                     # 存放区长边(红: 沿 x) [CAD dx=2000]
STOR_W   = 1000.0                     # 存放区短边(沿 y)      [CAD dz=1000]
SKY_AREA = 1200.0                     # 地面共享区(天空方块) 1200×1200
PILLAR_SH= 1000.0                     # 圣物柱附加共享区 1000×1000
TRANS_S  = 1000.0                     # 交接区 1000×1000
RETRY_L1 = 700.0                      # 一层重试区 700×700
SPOT     = 500.0                      # 建造点位 500×500
MUST_PD  = 270.0                      # 圣物柱直径(底座 Ø270)
MUST_PH  = 500.0                      # 圣物柱高
CENT_PD  = 270.0                      # 中央柱直径 Ø270
CENT_PH  = 800.0                      # 中央柱高
EARTH    = 350.0                      # 大地方块边长
SKY      = 200.0                      # 天空方块边长
MUST_D   = 200.0                      # 圣物(灵石)直径: CAD 用 Ø200; 规则 V1.1 为排球 Ø210

# ---- 比赛物体质量(kg) [规则 V1.1] -------------------------------------
EARTH_M  = 0.60                       # 大地方块 ≈600 g
SKY_M    = 0.22                       # 天空方块 ≈220 g
MUST_M   = 0.27                       # 圣物     ≈260–280 g

# ---- 栅栏/墙 高度·厚度 ------------------------------------------------
# [CAD] 1区栅栏 高 100、厚约 50; 2区栅栏 同高。没有官方数值,可改。
BOUND_H  = 100.0                      # 场地四周矮栏高      [CAD 1区栅栏 y25..125]
BOUND_T  = 50.0                       # 栏厚
DIV_H    = 100.0                      # 分隔栏高(地面/一层上)  [CAD y25..125 / 625..725]
DIV_T    = 50.0                       # 分隔栏厚
RAIL_H   = 100.0                      # 一层台面四周挡板高  [CAD 2区栅栏 y625..725]

# ==========================================================================
# 三、高程(mm)  —— [规则],与 CAD 装配换算后一致
# ==========================================================================
Z0       = 0.0                        # 地面顶面(= CAD y=25)
ZL1      = L1_H                       # 一层台面顶 = 600
ZL2      = L2_H                       # 二层台面顶 = 900
ZCENT    = L2_H + CENT_PH             # 中央柱顶 = 1700

# ==========================================================================
# 四、布局参数  ★★★ 想挪东西主要改这里 ★★★
#     每个元素都给注释:值来源 + 含义。坐标都是(x中心, y中心)mm。
# ==========================================================================
# ---- 布局开关 ----
DRAW_OBJECTS    = True   # 是否生成带物理的比赛物体(大地方块/天空方块/圣物)
DRAW_L1_RAIL    = True   # 一层台面四周的矮挡板
SHOW_L1_SHARED  = True   # 是否画 L1 共享区米黄块(边界处两处)

# ---- 一层/二层台面(位置由规则固定:同心居中) ----
# [CAD] 二层基座(6000×6000)与 3区(3000×3000)都在场地中心; 红蓝以 x=0 分界。

# ---- 地面区域(红方给一组, 蓝方自动镜像 x→-x) ------------------------
# 每个元组: (名称, cx, cy, 宽w(沿x), 深d(沿y), 颜色)
# ① 启动/重试区(地面): [CAD] 1区启动区 700×700, 两块, 在北侧两角
#    CAD: 红 (-5150,5150) (-4350,5150); 蓝 (4350,5150)(5150,5150)
START_RED   = [(-5150.,  5150.), (-4350.,  5150.)]
START_BLUE  = [( 4350.,  5150.), ( 5150.,  5150.)]

# ② 存放区: [CAD] 1区储存区 2000(沿x)×1000(沿y), 在南侧外角
#    红中心(-4500,-5000); 蓝镜像(4500,-5000)
STORE_RED   = (-4500., -5000.)
STORE_BLUE  = ( 4500., -5000.)

# ③ 地面共享区(12 天空方块, 5×5): [CAD] 1区-地面共享区 中心(0,4250)
SKY_GRID_C  = (0.,  4250.)

# ④ 圣物柱 + 附加共享区: [CAD] 1区-地面附加共享区 中心(0,-4250); 灵石基座同点
PILLAR_C    = (0., -4250.)
MUST_C      = (0., -4250.)            # 灵石球放置点(柱顶带凹槽, 微沉一点)

# ⑤ 圣物沉入凹槽量: [推断] 柱顶有个 Ø180 深100 的凹槽(规则), 球应放里面。
#    当前 v2 还没把凹槽造型做出来, 球默认"搁在柱顶平面", 下沉量先给 0,
#    避免开局被碰撞顶出去。以后加了凹槽碰撞面再把该值设成约 30。
MUST_SINK   = 0.0

# ---- 一层台面(L1)上的东西 -------------------------------------------
# ⑥ 一层重试区: [CAD] 2区重试区 700×700, 台面西南/东南, 中心 y=-2600
RETRY_L1_RED = (-1800., -2600.)
RETRY_L1_BLUE= ( 1800., -2600.)

# ⑦ L1 共享区(边界争夺点,米黄,共2块靠台面南北端):
#    [推断] CAD 没画这块色, 参照边界两个"骑线建造点"放的。可改/可关(SHOW_L1_SHARED)。
L1_SHARED_POS = [(0., -2700.), (0., 2700.)]     # 两块的中心
L1_SHARED_SZ  = 900.0                           # 边长

# ⑧ 一层台面边界上的分隔栏段(x=0, 挡在二层基座南北两侧):
#    [CAD] 匿名体: y 段 [1500,2200] 与 [-2200,-1500], 高100
DECK_DIV_Y   = [(1500., 2200.), (-2200., -1500.)]

# ⑨ 建造点位 ★ [CAD] 一层 6 个(台面 6000 内, 距边 300):
#    L1: (±2700,±2700) 四角 + (0,±2700) 两个骑线(可被双方争夺=共享)
#    二层 4 个(3000 内, 距边 250): L2:(±1250,±1250)
SPOTS_L1 = [(-2700.,-2700.), (   0.,-2700.), (2700.,-2700.),
            (-2700., 2700.), (   0., 2700.), (2700., 2700.)]
SPOTS_L2 = [(-1250.,-1250.), (1250.,-1250.), (-1250., 1250.), (1250., 1250.)]

# ⑩ 一层台阶(地面→一层 600): [CAD] 台阶-红 贴一层西/东外侧(y≈0 一带)
#     每侧一级结构在 x[-4000,-3000]; 级数按规则 4 级。宽度/位置可调。
L1_STAIR_Y = 0.0                     # 台阶中心 y
L1_STAIR_W = 1200.0                  # 单级宽(沿 y), 想加宽就加大(见 CAD 长条结构)

# ⑪ 二层台阶(一层顶→二层, 差300 = 两级): [CAD] 二层台阶 300×150×1000 贴台体西/东
L2_STAIR_Y = 0.0

# ⑫ 斜坡(地面→一层, 长3500): [规则] 有, 但 CAD 没画、且居中布局下
#     只剩外侧边可放; 先用开关关闭, 需要时打开并设 base/top 两点。
RAMP_ENABLED = False
# 红方斜坡: base=坡底(地面), top=坡顶(一层台面顶). 两点必须都在场地内、斜率约 10°。
# (下为一组可用示例, 位于一层西侧、斜跨场地; 仅供参考)
RAMP_RED_AB  = ((-4900.,  0.), (-3000.,  0.))   # [推断] 待确认
RAMP_BLUE_AB = (( 4900.,  0.), ( 3000.,  0.))

# ---- 交接区: 规则给 1000×1000, 但 CAD 里没有这块, 位置待确认 ----------
# [推断] 先放在一层台面内侧、正对各自台阶入口的地方(方便 BR 收、TR 送上台)。
TRANS_RED    = (-2250., 0.)
TRANS_BLUE   = ( 2250., 0.)
TRANS_HINT   = "交接区位置 CAD 未标, 建议对照官方图确认: 在台面上还是地面?"

# ---- 地面分隔栏小段(0 号中线, 地上): [CAD] 1区栅栏内两个匿名小段 ---------
GND_DIV_Y    = [(3000., 3650.), (-3750., -3000.)]

# ==========================================================================
# 五、天空方块初始布局(5×5, 镜像, 中心空) [规则 4.1.4 + 图形解码]
#     行: 从北(远)到南(近); 列: 从西(红)到东(蓝)。 'R'/'B' = 朝上颜色。
# ==========================================================================
SKY_MAT = [
    ["R", None, "R", None, "B"],   # 北行(远)
    [None, "R", None, "B", None],
    ["B", None, None, None, "R"],  # 中心格空
    [None, "B", None, "R", None],
    ["R", None, "B", None, "B"],   # 南行(近)
]
assert sum(1 for r in SKY_MAT for c in r if c) == 12

# ==========================================================================
# 六、SDF 小工具
# ==========================================================================
MM = 0.001
def mat(rgb, a=1.0):
    """把 RGB 颜色转成 SDF 材质字符串。"""
    r, g, b = rgb
    fmt = lambda v: f"{v/255:.3f}"
    return (f"<ambient>{fmt(r)} {fmt(g)} {fmt(b)} {a:.2f}</ambient>"
            f"<diffuse>{fmt(r)} {fmt(g)} {fmt(b)} {a:.2f}</diffuse>"
            f"<specular>0.04 0.04 0.04 1</specular><emissive>0 0 0 1</emissive>")

def box(name, cx, cy, cz, w, d, h, colour, static=True, rpy=(0., 0., 0.),
        collide=True, visual=True):
    """
    一个长方体 <model>。
    参数全用 mm;rpy 用弧度。cz 是箱体中心高度。
    collide=False → 只有视觉(标注线/贴图用)。
    visual =False → 只有碰撞(配合 CAD 网格外观时用,见 CAD_OVERLAY)。
    """
    col = ""
    if collide:
        col = (f"<collision name='col'><geometry><box>"
               f"<size>{w*MM} {d*MM} {h*MM}</size></box></geometry></collision>")
    vis = ""
    if visual:
        vis = (f"<visual name='vis'><geometry><box>"
               f"<size>{w*MM} {d*MM} {h*MM}</size></box></geometry>"
               f"<material>{mat(colour)}</material></visual>")
    return (f"<model name='{name}'>\n  <static>{str(static).lower()}</static>\n"
            f"  <pose>{cx*MM:.4f} {cy*MM:.4f} {cz*MM:.4f} "
            f"{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}</pose>\n"
            f"  <link name='link'>{col}{vis}</link>\n</model>")

def cyl(name, cx, cy, cz, r, h, colour, static=True):
    """圆柱 <model>(用于两根柱)。"""
    return (f"<model name='{name}'>\n  <static>{str(static).lower()}</static>\n"
            f"  <pose>{cx*MM:.4f} {cy*MM:.4f} {cz*MM:.4f} 0 0 0</pose>\n"
            f"  <link name='link'>"
            f"<collision name='col'><geometry><cylinder>"
            f"<radius>{r*MM}</radius><length>{h*MM}</length></cylinder></geometry></collision>"
            f"<visual name='vis'><geometry><cylinder>"
            f"<radius>{r*MM}</radius><length>{h*MM}</length></cylinder></geometry>"
            f"<material>{mat(colour)}</material></visual></link>\n</model>")

def sph(name, cx, cy, cz, r, colour, static=True):
    """球体(圣物)"""
    return (f"<model name='{name}'>\n  <static>{str(static).lower()}</static>\n"
            f"  <pose>{cx*MM:.4f} {cy*MM:.4f} {cz*MM:.4f} 0 0 0</pose>\n"
            f"  <link name='link'>"
            f"<inertial><mass>{MUST_M}</mass><inertia>"
            f"<ixx>{0.4*MUST_M*(r*r)/1e6:.6g}</ixx><iyy>{0.4*MUST_M*(r*r)/1e6:.6g}</iyy>"
            f"<izz>{0.4*MUST_M*(r*r)/1e6:.6g}</izz></inertia></inertial>"
            f"<collision name='col'><geometry><sphere>"
            f"<radius>{r*MM}</radius></sphere></geometry></collision>"
            f"<visual name='vis'><geometry><sphere>"
            f"<radius>{r*MM}</radius></sphere></geometry>"
            f"<material>{mat(colour)}</material></visual></link>\n</model>")

def cube(name, cx, cy, cz, s, mass, colour=None, top_bot=None):
    """
    物理方块(比赛物体)。
      * 大地方块: 给 colour(纯色)。
      * 天空方块: 给 top_bot=(顶色RGB, 底色RGB) —— 上半一色、下半一色,
        翻个面"朝上色"就变了, 与规则一致。
    返回值直接是 SDF 片段。s、cx…单位 mm。
    """
    I = (f"<inertial><mass>{mass}</mass><inertia>"
         f"<ixx>{mass*2*s*s/12e6:.6g}</ixx><iyy>{mass*2*s*s/12e6:.6g}</iyy>"
         f"<izz>{mass*2*s*s/12e6:.6g}</izz></inertia></inertial>")
    body = f"<box><size>{s*MM} {s*MM} {s*MM}</size></box>"
    if top_bot:
        half = f"<box><size>{s*MM} {s*MM} {s/2*MM:.5f}</size></box>"
        vis = (f"<visual name='up'><pose>0 0 {s/4*MM:.5f} 0 0 0</pose>"
               f"<geometry>{half}</geometry><material>{mat(top_bot[0])}</material></visual>"
               f"<visual name='dn'><pose>0 0 {-s/4*MM:.5f} 0 0 0</pose>"
               f"<geometry>{half}</geometry><material>{mat(top_bot[1])}</material></visual>")
    else:
        vis = (f"<visual name='vis'><geometry>{body}</geometry>"
               f"<material>{mat(colour)}</material></visual>")
    return (f"<model name='{name}'>\n  <static>false</static>\n"
            f"  <pose>{cx*MM:.4f} {cy*MM:.4f} {cz*MM:.4f} 0 0 0</pose>\n"
            f"  <link name='link'>{I}"
            f"<collision name='col'><geometry>{body}</geometry></collision>"
            f"{vis}</link>\n</model>")

# ---- 装饰层:只做视觉、不参与碰撞, 收集到 field_decor 一个静态模型里 ----
_decor = []
def decor_add(cx, cy, z_bottom, w, d, colour, thick=6.0):
    """在地面某处贴一块 w×d×thick 的彩色薄板, z_bottom=底面高度(mm)。"""
    _decor.append((cx, cy, z_bottom, w, d, thick, colour))
def decor_emit():
    parts = ["<model name='field_decor'><static>true</static>"
             "<pose>0 0 0 0 0 0</pose><link name='link'>"]
    for i, (cx, cy, zb, w, d, th, col) in enumerate(_decor):
        parts.append(
            f"<visual name='v{i}'><pose>{cx*MM:.4f} {cy*MM:.4f} {(zb+th/2)*MM:.4f} 0 0 0</pose>"
            f"<geometry><box><size>{w*MM} {d*MM} {th*MM}</size></box></geometry>"
            f"<material>{mat(col)}</material></visual>")
    parts.append("</link></model>")
    return "".join(parts)

# ==========================================================================
# 七、根据布局常量, 计算每个元素 → SDF
# ==========================================================================
def build():
    p = []
    p.append("""<?xml version='1.0' ?>
<sdf version='1.6'>
  <world name='robocon2027_arena'>
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1</real_time_factor>
    </physics>
    <scene><ambient>0.55 0.55 0.62 1</ambient>
      <background>0.24 0.26 0.30 1</background><shadows>true</shadows></scene>
    <light type='directional' name='sun'>
      <pose>0 0 40 0 0 0</pose>
      <diffuse>0.92 0.92 0.9 1</diffuse><specular>0.3 0.3 0.3 1</specular>
      <direction>-0.55 -0.8 -1.0</direction><cast_shadows>true</cast_shadows>
    </light>""")

    # ---------- 1) 整片场地: 木地板块(碰撞顶面 z=0) + 兜底平面 ----------
    p.append(box("field_base", 0, 0, -30, FIELD+600, FIELD+600, 60, BASE))
    p.append("<model name='catch_floor'><static>true</static>"
             "<pose>0 0 -0.25 0 0 0</pose><link name='link'>"
             "<collision name='col'><geometry><plane><normal>0 0 1</normal>"
             "<size>200 200</size></plane></geometry></collision></link></model>")

    # ---------- 2) 结构: 一层/二层台体(碰撞+外观) ----------
    # 一层 = 一个实心方块, 顶面 z=600, 东西向 6000。
    p.append(box("l1_platform", 0, 0, L1_H/2, L1, L1, L1_H, WOOD))
    # 二层 = 一个实心方块, 顶面 z=900, 3000×3000, 在一层正中央。
    p.append(box("l2_platform", 0, 0, L1_H + (L2_H-L1_H)/2, L2, L2, L2_H-L1_H, L2_GREY))

    # ---------- 3) 两根柱(圣物柱 地面、中央柱 二层) ----------
    mx, my = MUST_C
    p.append(cyl("mustika_pillar", mx, my, MUST_PH/2, MUST_PD/2, MUST_PH, DIV))
    if DRAW_OBJECTS:
        # 圣物(灵石) 球: 柱顶 + 半径 - 凹槽沉入量
        p.append(sph("mustika_ball", mx, my, MUST_PH + MUST_D/2 - MUST_SINK,
                     MUST_D/2, GOLD))
    p.append(cyl("central_pillar", 0, 0, ZL2 + CENT_PH/2, CENT_PD/2, CENT_PH, DIV))

    # ---------- 4) 场地四周矮栏 + 分隔栏 ----------
    for nm, (cx, cy, w, d) in {
        "pN": (0,  HLF+BOUND_T/2, FIELD+2*BOUND_T, BOUND_T),
        "pS": (0, -HLF-BOUND_T/2, FIELD+2*BOUND_T, BOUND_T),
        "pE": ( HLF+BOUND_T/2, 0, BOUND_T, FIELD+2*BOUND_T),
        "pW": (-HLF-BOUND_T/2, 0, BOUND_T, FIELD+2*BOUND_T)}.items():
        p.append(box("fence_"+nm, cx, cy, BOUND_H/2, w, d, BOUND_H, DIV))
    # 地面中线分隔小段 [CAD 1区栅栏]
    for i, (y0, y1) in enumerate(GND_DIV_Y):
        p.append(box(f"gdiv_{i}", 0, (y0+y1)/2, DIV_H/2, DIV_T, y1-y0, DIV_H, DIV))
    # 一层台面上分隔段 [CAD 匿名体 y1500..2200 / -2200..-1500]
    for i, (y0, y1) in enumerate(DECK_DIV_Y):
        p.append(box(f"ddiv_{i}", 0, (y0+y1)/2, ZL1+DIV_H/2, DIV_T, y1-y0, DIV_H, DIV))

    # ---------- 5+6) 登台结构(斜坡+平台+台阶) — 按 CAD "台阶-红/蓝" 实测重建 ----
    # CAD 里 "台阶-红" 其实是一整条贴着台面西外缘(x[-4000,-3000], 宽1000)的登台通道,
    # 沿 y(南北)分三段;蓝方在 x 正向镜像:
    #   南段  y∈[-2700, 800] : 3500 长斜坡, 从地面(z0)爬升到台面(z600)
    #   中段  y∈[ 800, 1800] : 高 600 的平台(与一层台面等高, 在此登上一层)
    #   北段  y∈[ 1800, 2700]: 三级台阶(顶450/300/150) 落回地面
    for xsgn in (-1, 1):
        side = "red" if xsgn < 0 else "blue"
        colr = STAIR_R if xsgn < 0 else STAIR_B
        Xc   = xsgn*3500.0                    # 通道中线 x=±3500
        # (a) 南段斜坡(斜板): 抬600、水平3500 → 沿 y 上升
        theta = math.atan2(600.0, 3500.0)     # ≈9.74°
        Lslab = 3550.0
        p.append(box(f"ramp_{side}", Xc, -950.0, 300.0, Lslab, 1000.0, 60.0,
                     RAMP_R if xsgn < 0 else RAMP_B, rpy=(0.0, -theta, math.pi/2)))
        # (b) 中段等高平台(600 高实心)
        p.append(box(f"plat_{side}", Xc, 1300.0, 300.0, 1000.0, 1000.0, 600.0, colr))
        # (c) 北段三级台阶(往下 450/300/150)
        for k, (cyy, top) in enumerate(((1950.0, 450.0), (2250.0, 300.0), (2550.0, 150.0))):
            p.append(box(f"stp_{side}_{k}", Xc, cyy, top/2, 1000.0, 300.0, top, colr))
        # 二层台阶(一层顶600→750 一级; 二层台体边=第2级) [CAD 二层台阶 300×150×1000]
        l2x = xsgn*(L2/2 + STEP_DEP/2)
        p.append(box(f"st2_{side}", l2x, L2_STAIR_Y, ZL1+STEP_H/2,
                     STEP_DEP, STEP_W, STEP_H, colr))

    # ---------- 7) 一层台面四周矮挡板(防物体/机器人跌落) ----------
    # 北/南边整条; 西/东边只留角段(中间 -2700..2700 是登台通道入口, 开口)
    if DRAW_L1_RAIL:
        for sgn in (-1, 1):                 # 北(+3000)/南(-3000) 沿 x 整条
            p.append(box(f"rail_ns{'+' if sgn > 0 else '-'}", 0, sgn*L1/2,
                         ZL1+RAIL_H/2, L1, RAIL_H, RAIL_H, L2_GREY))
        for xsgn in (-1, 1):                # 西/东边只做两角段
            for k, (ya, yb) in enumerate(((-L1/2, -2700.0), (2700.0, L1/2))):
                p.append(box(f"rail_we_{'-' if xsgn < 0 else '+'}_{k}",
                             xsgn*L1/2, (ya+yb)/2,
                             ZL1+RAIL_H/2, RAIL_H, yb-ya, RAIL_H, L2_GREY))

    # ---------- 8) 装饰/标注层(全部视觉, 无碰撞) ----------
    T = 6.0                              # 贴图厚度
    # 地面两半底色(微微抬高避免和地板块顶面 z-fight)
    decor_add(-HLF/2, 0, 1, HLF-1, FIELD-1, G_RED)
    decor_add( HLF/2, 0, 1, HLF-1, FIELD-1, G_BLUE)
    # 启动/重试区(地面)
    for (cx, cy) in START_RED:
        decor_add(cx, cy, 7, START_S-4, START_S-4, C_RED)
    for (cx, cy) in START_BLUE:
        decor_add(cx, cy, 7, START_S-4, START_S-4, C_BLUE)
    # 存放区(用色块标出)
    for (cx, cy, col) in ((STORE_RED[0], STORE_RED[1], C_RED),
                          (STORE_BLUE[0], STORE_BLUE[1], C_BLUE)):
        decor_add(cx, cy, 7, STOR_L, STOR_W, col)
    # 地面共享(天空方块区) & 圣物附加共享
    decor_add(SKY_GRID_C[0], SKY_GRID_C[1], 7, SKY_AREA-4, SKY_AREA-4, SHARED)
    decor_add(PILLAR_C[0],  PILLAR_C[1],  7, PILLAR_SH-4, PILLAR_SH-4, SHARED)
    # 一层台面两半底色
    decor_add(-L1/4, 0, ZL1+1, L1/2-1, L1-1, L1_RED)
    decor_add( L1/4, 0, ZL1+1, L1/2-1, L1-1, L1_BLUE)
    # 二层台面(整片灰色)
    decor_add(0, 0, ZL2+1, L2-1, L2-1, L2_GREY)
    # 一层重试区 + 交接区 + L1共享(米黄)
    for (cx, cy, col) in ((RETRY_L1_RED[0], RETRY_L1_RED[1], C_RED),
                          (RETRY_L1_BLUE[0], RETRY_L1_BLUE[1], C_BLUE),
                          (TRANS_RED[0], TRANS_RED[1], TR_R),
                          (TRANS_BLUE[0], TRANS_BLUE[1], TR_B)):
        decor_add(cx, cy, ZL1+7, RETRY_L1-4, RETRY_L1-4, col)
    if SHOW_L1_SHARED:
        for (cx, cy) in L1_SHARED_POS:
            decor_add(cx, cy, ZL1+7, L1_SHARED_SZ-4, L1_SHARED_SZ-4, SHARED)
    # 建造点位(绿)
    for (bx, by) in SPOTS_L1:
        decor_add(bx, by, ZL1+13, SPOT-4, SPOT-4, SPOT_C)
    for (bx, by) in SPOTS_L2:
        decor_add(bx, by, ZL2+13, SPOT-4, SPOT-4, SPOT_C)
    p.append(decor_emit())

    # ---------- 9) 比赛物体(物理) ----------
    if DRAW_OBJECTS:
        # 大地方块: 每队 20 个。放在各自存放区里: 5 列(沿 x)×2 行(沿 y), 每摞 2 个高
        n = 0
        for store, colour in ((STORE_RED, C_RED), (STORE_BLUE, C_BLUE)):
            xc, yc = store
            sp = EARTH + 25.0                # 方块中心间距(留 25 间隙)
            for ci in range(5):              # 5 列沿 x
                for ri in range(2):          # 2 行沿 y
                    x = xc + (ci - 2.0) * sp
                    y = yc + (ri - 0.5) * sp
                    for hi in range(2):      # 每摞 2 层
                        z = EARTH/2 + hi * (EARTH + 3.0)
                        p.append(cube(f"earth_{n}", x, y, z, EARTH, EARTH_M,
                                      colour=colour))
                        n += 1
        # 天空方块 5×5 网格(12 个, 中心空); 颜色按 SKY_MAT(顶/底各一色)
        pitch = SKY_AREA/5
        x0 = SKY_GRID_C[0] - SKY_AREA/2 + pitch/2
        y0 = SKY_GRID_C[1] + SKY_AREA/2 - pitch/2   # 第一行在北(远)
        n = 0
        for ri, row in enumerate(SKY_MAT):
            for ci, col in enumerate(row):
                if not col:
                    continue
                top = C_RED if col == "R" else C_BLUE
                bot = C_BLUE if col == "R" else C_RED
                p.append(cube(f"sky_{n}", x0 + ci*pitch, y0 - ri*pitch,
                              SKY/2, SKY, SKY_M, top_bot=(top, bot)))
                n += 1

    p.append("  </world>\n</sdf>\n")
    return "".join(p)

# ==========================================================================
# 八、俯视平面图(SVG), 方便你用浏览器和规则书/图纸核对
# ==========================================================================
def svg_plan():
    SC, ox, oy = 0.05, 560, 560
    def P(x, y): return ox + x*SC, oy - y*SC
    E = [f"<svg xmlns='http://www.w3.org/2000/svg' width='1120' height='1120' "
         f"viewBox='0 0 1120 1120'><rect width='1120' height='1120' fill='#fbfaf6'/>"]
    E.append("<text x='16' y='24' font-size='18' font-family='sans-serif'>"
             "ABU ROBOCON 2027 arena 俯视 (v2, 以 CAD 尺寸为准) — 对照图纸核对</text>")
    def rr(cx, cy, w, d, rgb, op=1, lab=None, fs=10):
        X, Y = P(cx, cy)
        E.append(f"<rect x='{P(cx-w/2,cy-d/2)[0]:.1f}' y='{P(cx-w/2,cy-d/2)[1]:.1f}' "
                 f"width='{w*SC:.1f}' height='{d*SC:.1f}' fill='rgb{rgb}' "
                 f"fill-opacity='{op}' stroke='#333' stroke-width='1'/>")
        if lab:
            E.append(f"<text x='{X:.1f}' y='{Y:.1f}' font-size='{fs}' "
                     f"text-anchor='middle' dominant-baseline='middle'>{html.escape(lab)}</text>")
    # 外框/两半
    rr(0, 0, FIELD, FIELD, (0,0,0), 0)
    rr(-HLF/2, 0, HLF-2, FIELD-2, G_RED, .45, "RED")
    rr( HLF/2, 0, HLF-2, FIELD-2, G_BLUE, .45, "BLUE")
    # 一层/二层台面框
    rr(0, 0, L1, L1, (140,120,100), .25)
    rr(0, 0, L2, L2, L2_GREY, .9, "L2")
    # 启动区(地面) & 存放区
    for (cx, cy) in START_RED+START_BLUE:
        rr(cx, cy, START_S, START_S, (255,0,0) if cx<0 else (0,0,255), .8, "S")
    for (cx, cy, col) in ((STORE_RED[0], STORE_RED[1], C_RED), (STORE_BLUE[0], STORE_BLUE[1], C_BLUE)):
        rr(cx, cy, STOR_L, STOR_W, col, .6, "STOR")
    # 共享区
    rr(SKY_GRID_C[0], SKY_GRID_C[1], SKY_AREA, SKY_AREA, SHARED, .9, "sky")
    rr(PILLAR_C[0], PILLAR_C[1], PILLAR_SH, PILLAR_SH, SHARED, .9, "M")
    # 重试/交接(一层)
    for (cx, cy) in (RETRY_L1_RED, RETRY_L1_BLUE):
        rr(cx, cy, RETRY_L1, RETRY_L1, (200,200,200), .6)
    # 建造点
    for (bx, by) in SPOTS_L1 + SPOTS_L2:
        rr(bx, by, SPOT, SPOT, SPOT_C, 1)
    # 天空方块标记
    pitch = SKY_AREA/5
    x0 = SKY_GRID_C[0] - SKY_AREA/2 + pitch/2
    y0 = SKY_GRID_C[1] + SKY_AREA/2 - pitch/2
    for ri, row in enumerate(SKY_MAT):
        for ci, col in enumerate(row):
            if not col: continue
            X, Y = P(x0+ci*pitch, y0-ri*pitch)
            E.append(f"<rect x='{X-SKY*SC/2:.1f}' y='{Y-SKY*SC/2:.1f}' "
                     f"width='{SKY*SC:.1f}' height='{SKY*SC:.1f}' "
                     f"fill='{'#df2222' if col=='R' else '#3300ee'}'/>")
    # 柱
    for (px, py, lab) in ((MUST_C[0], MUST_C[1], "M"), (0, 0, "C")):
        X, Y = P(px, py)
        E.append(f"<circle cx='{X:.1f}' cy='{Y:.1f}' r='{135*SC:.1f}' fill='rgb{DIV}'/>"
                 f"<text x='{X:.1f}' y='{Y:.1f}' font-size='10' text-anchor='middle'"
                 f" dominant-baseline='middle' fill='white'>{lab}</text>")
    E.append("</svg>")
    return "".join(E)

# ==========================================================================
def main():
    os.makedirs(os.path.dirname(OUT_WORLD), exist_ok=True)
    os.makedirs(os.path.dirname(OUT_SVG), exist_ok=True)
    world = build()
    with open(OUT_WORLD, "w") as f:
        f.write(world)
    with open(OUT_SVG, "w") as f:
        f.write(svg_plan())
    print(f"OK  生成 {os.path.relpath(OUT_WORLD)}  (模型数 {world.count('<model')})")
    print(f"    生成 {os.path.relpath(OUT_SVG)}")

if __name__ == "__main__":
    main()
