#!/usr/bin/env python3
"""Inspect the running sensor/TF/mapping chain; run inside the ROS 2 VM."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener


class MappingCheck(Node):
    def __init__(self):
        super().__init__('rc_mapping_check', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.count = dict(clock=0, cloud=0, scan=0, odom=0, map=0)
        self.latest = {}
        self.finite_cloud = 0
        self.finite_scan = 0
        self.subscriptions_kept = []
        for key, msg_type, topic in [
            ('clock', Clock, '/clock'), ('cloud', PointCloud2, '/livox/lidar/pointcloud'),
            ('scan', LaserScan, '/scan'), ('odom', Odometry, '/odom')]:
            self.subscriptions_kept.append(self.create_subscription(
                msg_type, topic, lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data))
        self.subscriptions_kept.append(self.create_subscription(
            OccupancyGrid, '/map', lambda msg: self.receive('map', msg),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

    def receive(self, key, msg):
        self.count[key] += 1
        self.latest[key] = msg
        if key == 'cloud':
            self.finite_cloud = sum(1 for _ in point_cloud2.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True))
        elif key == 'scan':
            self.finite_scan = sum(math.isfinite(value) and msg.range_min <= value <= msg.range_max
                                   for value in msg.ranges)

    def report(self):
        print('Messages received (20 seconds wall time):', self.count)
        problems = []
        for key, topic in [('clock', '/clock'), ('cloud', '/livox/lidar/pointcloud'),
                           ('scan', '/scan'), ('odom', '/odom'), ('map', '/map')]:
            if not self.count[key]:
                problems.append('No messages: ' + topic)
        if 'cloud' in self.latest:
            cloud = self.latest['cloud']
            print('Cloud frame:', cloud.header.frame_id,
                  'points:', cloud.width * cloud.height, 'finite:', self.finite_cloud)
            if not self.finite_cloud:
                problems.append('Cloud has no valid returns: check ray plugin, collisions, range')
            if 'clock' in self.latest:
                stamp = cloud.header.stamp
                clock = self.latest['clock'].clock
                delta = abs(stamp.sec + stamp.nanosec * 1e-9 - clock.sec - clock.nanosec * 1e-9)