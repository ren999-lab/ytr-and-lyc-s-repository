//
// Created by lfc on 2021/2/28.
//

#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_ros/node.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "livox_ode_multiray_shape.h"

namespace gazebo
{
   // Livox 扫描模式 CSV 中的一条记录。角度均为弧度；time 为扫描模式提供的
   // 单点时间信息。
   struct AviaRotateInfo
   {
      double time;
      double azimuth;
      double zenith;
   };

   // Gazebo 射线传感器插件：复现 Livox 的非重复扫描模式。插件复用固定数量的
   // ODE 射线，在每个扫描窗口更新其方向，并同时发布 Livox CustomMsg 和 PointCloud2。
   class LivoxPointsPlugin : public RayPlugin
   {
   public:
      LivoxPointsPlugin();

      virtual ~LivoxPointsPlugin();

      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

   private:
      ignition::math::Angle AngleMin() const;

      ignition::math::Angle AngleMax() const;

      double GetAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double AngleResolution() const;

      double GetRangeMin() const GAZEBO_DEPRECATED(7.0);

      double RangeMin() const;

      double GetRangeMax() const GAZEBO_DEPRECATED(7.0);

      double RangeMax() const;

      double GetRangeResolution() const GAZEBO_DEPRECATED(7.0);

      double RangeResolution() const;

      int GetRayCount() const GAZEBO_DEPRECATED(7.0);

      int RayCount() const;

      int GetRangeCount() const GAZEBO_DEPRECATED(7.0);

      int RangeCount() const;

      int GetVerticalRayCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRayCount() const;

      int GetVerticalRangeCount() const GAZEBO_DEPRECATED(7.0);

      int VerticalRangeCount() const;

      ignition::math::Angle VerticalAngleMin() const;

      ignition::math::Angle VerticalAngleMax() const;

      double GetVerticalAngleResolution() const GAZEBO_DEPRECATED(7.0);

      double VerticalAngleResolution() const;

   protected:
      // 每次 Gazebo 传感器更新时：更新复用射线，并填充输出的 LaserScan、
      // Livox CustomMsg 与 PointCloud2 三种消息。
      virtual void OnNewLaserScans();

   private:
      // 将当前扫描窗口映射到可复用的 ODE 射线。points_pair 保存每个输出点
      // 所对应的射线索引及 CSV 扫描方向。
      void InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                          boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape);

      // 重置传统 Gazebo LaserScan 的元数据和距离数组。
      void InitializeScan(msgs::LaserScan *&scan);

      void SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame, const std::string &child_frame);

      boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
      gazebo::physics::CollisionPtr laserCollision;
      physics::EntityPtr parentEntity;
      transport::PublisherPtr scanPub;

      sdf::ElementPtr sdfPtr;
      transport::NodePtr node;
      msgs::LaserScanStamped laserMsg;
      gazebo::sensors::SensorPtr raySensor;
      std::vector<AviaRotateInfo> aviaInfos;

      gazebo_ros::Node::SharedPtr node_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub;
      rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub;
      
      std::string parent_name;
      std::string child_name;
      int64_t samplesStep = 0;
      int64_t currStartIndex = 0;
      int64_t maxPointSize = 1000;
      int64_t downSample = 1;

      // SDF 中配置的量程上下限，单位为米。CSV 扫描模式循环使用，
      // 每发布一帧后 currStartIndex 都前进 samplesStep。
      double maxDist = 400.0;
      double minDist = 0.1;
   };

} // namespace gazebo

#endif // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H


