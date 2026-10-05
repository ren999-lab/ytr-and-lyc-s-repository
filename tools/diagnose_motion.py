#!/usr/bin/env python3
"""Read-only Gazebo/IMU/LIO yaw comparison. Does not send motion commands."""
import argparse
import bisect
import json
import math
import time


def unwrap(samples):
    result = []
    for stamp, angle in sorted(samples):
        if result:
            angle = result[-1][1] + math.atan2(
                math.sin(angle - result[-1][1]), math.cos(angle - result[-1][1]))
        result.append((stamp, angle))
    return result


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def compare_yaw(ground, estimated):
    """Align timestamps and remove the constant initial frame rotation."""
    ground, estimated = unwrap(ground), unwrap(estimated)
    times = [p[0] for p in ground]
    differences = []
    for stamp, angle in estimated:
        i = bisect.bisect_left(times, stamp)
        if i < len(times) and abs(times[i] - stamp) < 1e-6:
            actual = ground[i][1]
        elif 0 < i < len(times) and times[i] - times[i - 1] < 0.3:
            fraction = (stamp - times[i - 1]) / (times[i] - times[i - 1])
            actual = ground[i - 1][1] + fraction * (ground[i][1] - ground[i - 1][1])
        else:
            continue
        differences.append(angle - actual)
    if len(differences) < 2:
        return None
    errors = [abs(math.degrees(d - differences[0])) for d in differences]
    return {'paired_samples': len(errors), 'max_relative_yaw_error_deg': max(errors),
            'final_relative_yaw_error_deg': math.degrees(differences[-1] - differences[0])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=45, help='wall seconds to observe')
    parser.add_argument('--output', help='optional JSON report including observations')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be positive')

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage

    class Observer(Node):
        def __init__(self):
            super().__init__('rc_motion_diagnosis', parameter_overrides=[
                Parameter('use_sim_time', value=True)])
            self.clock = None
            self.command = None
            self.rows = {k: [] for k in ('gazebo', 'lio', 'imu', 'cloud', 'command')}
            self.parents = set()
            self.frames = {}
            self.subs = [
                self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data),
                self.create_subscription(Twist, '/cmd_vel_chassis', self.on_command, qos_profile_sensor_data),
                self.create_subscription(Odometry, '/odom', lambda m: self.on_odom('gazebo', m), qos_profile_sensor_data),
                self.create_subscription(Odometry, '/lio/odom', lambda m: self.on_odom('lio', m), qos_profile_sensor_data),
                self.create_subscription(Imu, '/livox/imu', self.on_imu, qos_profile_sensor_data),
                self.create_subscription(PointCloud2, '/livox/lidar/pointcloud', self.on_cloud, qos_profile_sensor_data),
                self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data),
            ]

        @staticmethod
        def seconds(stamp):
            return stamp.sec + stamp.nanosec * 1e-9

        def on_clock(self, msg):
            self.clock = self.seconds(msg.clock)

        def on_command(self, msg):
            if self.clock is not None:
                self.command = [self.clock, msg.linear.x, msg.linear.y, msg.angular.z]
                self.rows['command'].append(self.command)

        def on_odom(self, key, msg):
            stamp = self.seconds(msg.header.stamp)
            straight = bool(self.command is not None and
                            0 <= stamp - self.command[0] <= 0.5 and
                            abs(self.command[1]) > 0.05 and
                            abs(self.command[2]) < 1e-6 and abs(self.command[3]) < 1e-6)
            p = msg.pose.pose
            self.rows[key].append([stamp, yaw(p.orientation), p.position.x,
                                   p.position.y, msg.twist.twist.angular.z, straight])
            self.frames[key] = [msg.header.frame_id, msg.child_frame_id]

        def on_imu(self, msg):
            a, w = msg.linear_acceleration, msg.angular_velocity
            self.rows['imu'].append([self.seconds(msg.header.stamp), w.z,
                                    math.sqrt(a.x*a.x + a.y*a.y + a.z*a.z), w.x, w.y])
            self.frames['imu'] = msg.header.frame_id

        def on_cloud(self, msg):
            stamp = self.seconds(msg.header.stamp)
            age = None if self.clock is None else self.clock - stamp
            self.rows['cloud'].append([stamp, age])
            self.frames['cloud'] = msg.header.frame_id

        def on_tf(self, msg):
            for tf in msg.transforms:
                if tf.child_frame_id == 'dummy':
                    self.parents.add(tf.header.frame_id)

        def report(self):
            r = {'counts': {k: len(v) for k, v in self.rows.items()},
                 'frames': self.frames, 'dummy_dynamic_parents': sorted(self.parents)}
            ground = [(p[0], p[1]) for p in self.rows['gazebo']]
            r['lio_vs_gazebo'] = compare_yaw(ground, [(p[0], p[1]) for p in self.rows['lio']])
            segments, current = [], []
            for row in self.rows['gazebo'] + [[0, 0, 0, 0, 0, False]]:
                if not row[5] or (current and row[0] - current[-1][0] > 0.3):
                    if len(current) > 1 and current[-1][0] - current[0][0] >= 0.5:
                        angles = [v for _, v in unwrap([(p[0], p[1]) for p in current])]
                        segments.append({'simulation_seconds': current[-1][0] - current[0][0],
                                         'heading_span_deg': math.degrees(max(angles) - min(angles))})
                    current = []
                if row[5]:
                    current.append(row)
            r['gazebo_during_straight_commands'] = segments
            imu = self.rows['imu']
            integrated = []
            if imu:
                integrated = [(imu[0][0], 0.0)]
                for previous, row in zip(imu, imu[1:]):
                    dt = row[0] - previous[0]
                    if not 0 < dt < 0.1:
                        integrated = []
                        break
                    integrated.append((row[0], integrated[-1][1] + dt*(previous[1]+row[1])/2))
            # Gyro-z integration approximates yaw only while the chassis is level.
            r['imu_z_vs_gazebo_level_chassis_only'] = compare_yaw(ground, integrated)
            r['observations'] = self.rows
            return r

    rclpy.init()
    node = Observer()
    print('只读采样开始。请先静止，再在安全空地低速直行、停下；本脚本不发送控制命令。', flush=True)
    try:
        end = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        report = node.report()
        print(json.dumps({k: v for k, v in report.items() if k != 'observations'},
                         ensure_ascii=False, indent=2))
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
