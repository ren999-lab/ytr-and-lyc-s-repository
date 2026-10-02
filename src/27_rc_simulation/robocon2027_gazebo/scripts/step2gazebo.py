#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STEP → Gazebo 转换工具 (基于 gmsh)
=====================================================================
把 SolidWorks 导出的总装配 STEP 直接转成 Gazebo 可加载的场地网格,
得到一个和图纸 1:1 的静态外观模型(单色), 用于:
  * 对照核对自己参数化的场地(worlds/robocon2027_arena.world)有没有偏差;
  * 或作为机器人仿真里的"真实外观底层"。

用法:
  python3 step2gazebo.py                    # 用默认参数转 2027场地总装配.STEP
  python3 step2gazebo.py <输入.step> [细度m] [输出目录]

说明:
  * 网格细分用 gmsh, 会装到 ~/.local(首次会 pip 装 gmsh)。
  * 颜色: 该 STEP 是 AP203(不带颜色), 转出来是单色/灰。要上色请看
    gen_arena.py(按规则书颜色贴色块)。
  * 坐标系换算: CAD 是 y 向上;本工具输出时把
      CAD(x, y↑, z北)  →  Gazebo(x, z, y-25)
    即把 CAD 地板块顶面(y=25)放到 Gazebo 的 z=0。
  * 碰撞: 默认不开(大三角网格静态碰撞会让物理变慢且易抖)。
    需要真碰撞时把下面 USE_COLLISION 改为 True。
"""

import os, sys, time, math, struct

HERE     = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STEP = "/home/ytr/2027RC场地总装配-by风雅荷/2027场地总装配.STEP"
FLOOR_TOP_CAD_Y = 25.0        # CAD 地板块顶面的 y(mm);转完后这里 = Gazebo z=0

USE_COLLISION = False         # True = 网格也当碰撞(会慢); False = 只看
MATERIAL_STR  = "0.65 0.65 0.66 1"     # 单色外观(浅灰)

def main():
    step = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_STEP
    char = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10   # 网格细度(米), 越小越精细越慢
    outdir = sys.argv[3] if len(sys.argv) > 3 else os.path.normpath(
        os.path.join(HERE, "..", "models", "cad_arena"))

    # ---- 确保有 gmsh ----
    try:
        import gmsh
    except ImportError:
        print("[step2gazebo] 安装 gmsh ...")
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", "gmsh"])
        import gmsh

    os.makedirs(os.path.join(outdir, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(outdir, "..", "..", "worlds"), exist_ok=True)

    t0 = time.time()
    gmsh.initialize()
    gmsh.option.setNumber("General.Verbosity", 1)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 0.001)
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", char)
    gmsh.open(step)

    # ---------- 过滤:只网格化"结构体",扔掉贴图/物件 ----------
    # 原因: CAD 里有很多 0.1 mm 厚的贴图板(启动/重试/建造点…)和我们在
    # 仿真里自己会生成的物件(天空方块/灵石球), 一起网格化会很慢且会重复。
    def cad_bbox(d, t):
        x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(d, t)
        return x0, y0, z0, x1, y1, z1
    remove = []
    kept = 0
    for (d, t) in gmsh.model.getEntities(3):
        name = gmsh.model.getEntityName(d, t) or ""
        x0, y0, z0, x1, y1, z1 = cad_bbox(d, t)
        ex, ey, ez = x1-x0, y1-y0, z1-z0
        minT, maxT = min(ex, ey, ez), max(ex, ey, ez)
        cy_cad = (y0+y1)/2
        # 1) 极薄的贴图板 / 色块(启动/重试/储存/建造点/共享区…)
        if minT < 5.0:
            remove.append((d, t)); continue
        # 2) 圣物球(仿真里已放金色灵石)
        if "灵石" in name:
            remove.append((d, t)); continue
        # 3) 落在地面的小方块(匿名天空方块等, 仿真里自己生成并上色)
        if not name and maxT <= 260.0 and cy_cad <= 260.0:
            remove.append((d, t)); continue
        kept += 1
    for (d, t) in remove:
        gmsh.model.occ.remove([(d, t)])
    gmsh.model.occ.synchronize()
    n_vol = len(gmsh.model.getEntities(3))
    print(f"[step2gazebo] 原始 {kept+len(remove)} 个体, 过滤后保留 {kept} 个结构体"
          f"(细度 {char} m) ...")
    gmsh.model.mesh.generate(2)                     # 只生成表面网格(外观)
    tmp = os.path.join(outdir, "meshes", "_raw.stl")
    gmsh.write(tmp)                                 # gmsh 直接导 STL
    gmsh.finalize()
    print(f"[step2gazebo] gmsh 网格化完成, 用时 {time.time()-t0:.1f}s")

    # ---- 坐标变换到 Gazebo 系 + 写最终 STL(ASCII 转一遍足够) ----
    raw = open(tmp, "r", errors="ignore").read()
    out_stl = os.path.join(outdir, "meshes", "arena.stl")
    ntri = 0
    minc = [1e9]*3; maxc = [-1e9]*3
    with open(out_stl, "w") as f:
        f.write("solid cad_arena\n")
        for line in raw.splitlines():
            if line.lstrip().startswith("vertex"):
                p = list(map(float, line.split()[1:4]))
                gx = p[0]                 # CAD x
                gy = p[2]                 # CAD z(北)
                gz = p[1] - FLOOR_TOP_CAD_Y   # CAD y 顶面归 0
                for i, v in enumerate((gx, gy, gz)):
                    minc[i] = min(minc[i], v); maxc[i] = max(maxc[i], v)
                f.write(f"  vertex {gx:.3f} {gy:.3f} {gz:.3f}\n")
            else:
                if line.lstrip().startswith("facet"):
                    ntri += 1
                f.write(line)
        f.write("endsolid cad_arena\n")
    os.remove(tmp)
    sz = os.path.getsize(out_stl)/1e6
    print(f"[step2gazebo] 已写 {out_stl}  ({sz:.1f} MB, 三角形 {ntri})")
    print(f"[step2gazebo] 包围盒(mm): x[{minc[0]:.0f},{maxc[0]:.0f}]  "
          f"y[{minc[1]:.0f},{maxc[1]:.0f}]  z[{minc[2]:.0f},{maxc[2]:.0f}]")

    # ---- 写 model.sdf / model.config(便于 <include> 用) ----
    coll = ("<collision name='mesh'>"
            "<geometry><mesh><uri>model://cad_arena/meshes/arena.stl</uri></mesh></geometry>"
            "</collision>") if USE_COLLISION else ""
    sdf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='cad_arena'>
    <static>true</static>
    <link name='link'>{coll}
      <visual name='mesh'>
        <geometry><mesh><uri>model://cad_arena/meshes/arena.stl</uri></mesh></geometry>
        <material><ambient>{MATERIAL_STR}</ambient><diffuse>{MATERIAL_STR}</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""
    open(os.path.join(outdir, "model.sdf"), "w").write(sdf)
    open(os.path.join(outdir, "model.config"), "w").write(
        "<?xml version='1.0'?>\n<model>\n  <name>cad_arena</name>\n"
        "  <version>1.0</version>\n  <sdf version='1.6'>model.sdf</sdf>\n</model>\n")

    # ---- 生成一个单独的世界文件: arena_cad.world(只看 CAD 外观) ----
    worlddir = os.path.normpath(os.path.join(HERE, "..", "worlds"))
    # 用绝对路径的 file:// uri, 不依赖 GAZEBO_MODEL_PATH
    uri = "file://" + out_stl
    wf = f"""<?xml version='1.0'?>
<sdf version='1.6'>
  <world name='cad_arena'>
    <physics type='ode'><max_step_size>0.001</max_step_size></physics>
    <scene><ambient>0.55 0.55 0.6 1</ambient><background>0.24 0.26 0.30 1</background></scene>
    <light type='directional' name='sun'><pose>0 0 40 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.6 -0.8 -1</direction></light>
    <model name='cad_arena'><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name='link'>
        <visual name='mesh'><geometry><mesh><uri>{uri}</uri></mesh></geometry>
          <material><ambient>{MATERIAL_STR}</ambient><diffuse>{MATERIAL_STR}</diffuse></material></visual>
      </link>
    </model>
  </world>
</sdf>
"""
    wpath = os.path.join(worlddir, "arena_cad.world")
    open(wpath, "w").write(wf)
    print(f"[step2gazebo] 已生成世界  {wpath}\n"
          f"  想看 CAD 原貌: gzserver {wpath}   (或 ros2 run gazebo_ros spawn 对比)\n"
          f"  说明: 这是单色外观参考层; 真机物理/色彩请用 gen_arena.py 参数化场地。")

if __name__ == "__main__":
    main()
