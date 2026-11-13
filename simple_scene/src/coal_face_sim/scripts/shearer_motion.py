#!/usr/bin/env python3
"""Kinematic reciprocation; ground truth is read independently from Gazebo."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry


class ShearerMotion(Node):
    def __init__(self):
        super().__init__('shearer_motion')
        for name, value in [('entity_name', 'shearer'), ('y_min', -4.0),
                            ('y_max', 4.0), ('period', 40.0), ('x', 0.0),
                            ('z', 0.4005), ('rate', 50.0)]:
            self.declare_parameter(name, value)
        self.cfg = {p: self.get_parameter(p).value for p in
                    ['entity_name', 'y_min', 'y_max', 'period', 'x', 'z', 'rate']}
        c = self.cfg
        if not all(math.isfinite(c[k]) for k in c if k != 'entity_name'):
            raise ValueError('Motion parameters must be finite')
        if c['y_min'] >= c['y_max'] or c['period'] <= 0 or c['rate'] <= 0:
            raise ValueError('Require y_min < y_max, period > 0, rate > 0')
        # This launch uses one 20 m tile, with a 2 m long body.
        if c['y_min'] < -8 or c['y_max'] > 8 or abs(c['x']) > 1:
            raise ValueError('Trajectory exceeds the conservative single-tile limits')
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.reference = self.create_publisher(Odometry, '/shearer/reference', 10)
        self.truth_sub = self.create_subscription(
            Odometry, '/ground_truth', self.on_truth, qos_profile_sensor_data)
        self.last_truth = None
        self.epoch = None
        self.last_time = None
        self.pending = None
        self.request_time = None
        self.failed = False
        self.timer = self.create_timer(1.0 / c['rate'], self.tick)
        self.get_logger().info('Waiting for Gazebo state service and /ground_truth')

    def on_truth(self, msg):
        self.last_truth = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if self.failed or t <= 0:
            return
        if self.last_time is not None and t < self.last_time:
            self.epoch = None
            self.last_truth = None
            self.get_logger().info('Simulation time reset; restarting trajectory')
        self.last_time = t
        if self.pending is not None:
            if not self.pending.done():
                if t - self.request_time > 2.0:
                    self.failed = True
                    self.get_logger().error('State service timed out; control stopped')
                return
            try:
                if not self.pending.result().success:
                    raise RuntimeError('Gazebo rejected entity state')
            except Exception as exc:
                self.failed = True
                self.get_logger().error(str(exc))
                return
            self.pending = None
        if not self.client.service_is_ready() or self.last_truth is None:
            return
        if abs(t - self.last_truth) > 0.5:
            return
        if self.epoch is None:
            self.epoch = t
            self.get_logger().info(
                f"Motion started: y={self.cfg['y_min']}..{self.cfg['y_max']} m, "
                f"period={self.cfg['period']} s, cosine profile")
        c = self.cfg
        omega = 2.0 * math.pi / c['period']
        phase = omega * (t - self.epoch)
        center = (c['y_min'] + c['y_max']) / 2.0
        amplitude = (c['y_max'] - c['y_min']) / 2.0
        y = center - amplitude * math.cos(phase)
        vy = amplitude * omega * math.sin(phase)
        request = SetEntityState.Request()
        request.state.name = c['entity_name']
        request.state.reference_frame = 'world'
        request.state.pose.position.x = c['x']
        request.state.pose.position.y = y
        request.state.pose.position.z = c['z']
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.y = vy
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose = request.state.pose
        msg.twist.twist = request.state.twist
        self.reference.publish(msg)
        self.pending = self.client.call_async(request)
        self.request_time = t


def main():
    rclpy.init()
    node = ShearerMotion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
