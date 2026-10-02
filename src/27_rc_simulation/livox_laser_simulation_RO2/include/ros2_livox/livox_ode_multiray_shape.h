//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
#define SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/util/system.hh>
#include <gazebo/ode/common.h>
#include <ignition/math6/ignition/math.hh>

namespace gazebo{
namespace physics{
class GZ_PHYSICS_VISIBLE LivoxOdeMultiRayShape : public MultiRayShape{
    // Gazebo 原生 MultiRayShape 创建的是固定规则网格。本类将射线暴露给
    // Livox 插件，使其能够按 CSV 扫描窗口更新方向，同时沿用 ODE 碰撞检测。
    /// \brief 构造函数。
    /// \param[in] _parent 父碰撞对象。
    public: explicit LivoxOdeMultiRayShape(CollisionPtr _parent);

    /// \brief 析构函数。
    public: virtual ~LivoxOdeMultiRayShape();

    /// \brief 对全部射线执行 ODE 碰撞检测。遍历 ODE 空间时会持有
    /// Gazebo 的物理更新互斥锁。
    public: virtual void UpdateRays();

    /// \brief 从传感器 SDF 读取射线几何参数和量程限制。
    public: virtual void Init();

    // 返回可修改的射线数组：Livox 插件会在测试下一扫描窗口前更新每条
    // 射线的起点和终点。
    public: std::vector<RayShapePtr> &RayShapes(){return rays;}
    /// \brief 射线与场景物体的相交回调。
    /// \param[in] _data 当前 LivoxOdeMultiRayShape 实例。
    /// \param[in] _o1 第一碰撞几何体。
    /// \param[in] _o2 第二碰撞几何体。
    private: static void UpdateCallback(void *_data, dGeomID _o1,
                                        dGeomID _o2);

    /// \brief 向碰撞对象添加一条射线。
    /// \param[in] _start 射线起点。
    /// \param[in] _end 射线终点。
    public: void AddRay(const ignition::math::Vector3d &_start,
                           const ignition::math::Vector3d &_end);
    /// \brief 包含射线子空间的父空间，用于与世界碰撞空间进行检测。
    private: dSpaceID superSpaceId;

    /// \brief 仅存放传感器射线的 ODE 子空间。
    private: dSpaceID raySpaceId;

 private:
    std::vector<RayShapePtr> livoxRays;
};
}
}


#endif  // SRC_GAZEBO_LIVOX_ODE_MULTIRAY_SHAPE_H
