#!/usr/bin/env python3

"""
straightness_test.py

Generic straight-line motion test using odom + IMU.

Parameters:
- linear_speed (positive = forward, negative = backward)
- test_name (used for JSON output)

Forward Test:
-------------------------------------------------------------------------
ros2 run tb3_odom_imu_validation straightness_test \
--ros-args -p linear_speed:=0.08 -p test_name:=forward_straightness
-------------------------------------------------------------------------
Backward Test:
-------------------------------------------------------------------------
ros2 run tb3_odom_imu_validation straightness_test \
--ros-args -p linear_speed:=-0.08 -p test_name:=backward_straightness
-------------------------------------------------------------------------
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tb3_odom_imu_validation.result_utils import append_result


# Defaults
DEFAULT_SPEED = 0.08
TARGET_DISTANCE = 1.0
LATERAL_DRIFT_TOL = 0.05
HEADING_DRIFT_TOL_DEG = 5.0
SETTLE_TIME = 1.0


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class StraightnessTest(Node):
    def __init__(self):
        super().__init__('straightness_test')

        # ===== Parameters =====
        self.declare_parameter('linear_speed', DEFAULT_SPEED)
        self.declare_parameter('test_name', 'straightness_test')

        self.linear_speed = self.get_parameter('linear_speed').value
        self.test_name = self.get_parameter('test_name').value

        # ===== ROS interfaces =====
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # ===== State =====
        self.current_x = None
        self.current_y = None
        self.current_imu_yaw = None

        self.start_x = None
        self.start_y = None
        self.start_imu_yaw = None

        self.phase = 'waiting_for_data'
        self.phase_start_time = time.time()
        self.done = False
        self.finish_time = None
        self.wait_log_time = 0.0

        self.timer = self.create_timer(0.05, self.loop)

        direction = "forward" if self.linear_speed > 0 else "backward"

        self.get_logger().info(f'Starting {self.test_name}')
        self.get_logger().info(f'Direction: {direction}')
        self.get_logger().info(f'Speed: {self.linear_speed:.2f} m/s')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def publish_cmd(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def data_ready(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_imu_yaw is not None
        )

    def finish_and_exit(self):
        self.publish_cmd()

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance = math.sqrt(dx * dx + dy * dy)

        lateral_drift = dy
        heading_change = math.degrees(
            normalize_angle(self.current_imu_yaw - self.start_imu_yaw)
        )

        lateral_ok = abs(lateral_drift) <= LATERAL_DRIFT_TOL
        heading_ok = abs(heading_change) <= HEADING_DRIFT_TOL_DEG
        distance_ok = distance >= TARGET_DISTANCE * 0.98

        status = 'PASS' if (lateral_ok and heading_ok and distance_ok) else 'FAIL'

        self.get_logger().info('=== Straightness Results ===')
        self.get_logger().info(f'Distance: {distance:.3f} m')
        self.get_logger().info(f'Lateral drift: {lateral_drift:.3f} m')
        self.get_logger().info(f'Heading drift: {heading_change:.2f} deg')
        self.get_logger().info(f'Result: {status}')

        append_result(
            test_name=self.test_name,
            status=status,
            measurement=f'{distance:.3f} m',
            notes=f'drift={lateral_drift:.3f}m, heading={heading_change:.2f}deg'
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                rclpy.shutdown()
            return

        if self.phase == 'waiting_for_data':
            if self.data_ready():
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_imu_yaw = self.current_imu_yaw

                self.phase = 'settling'
                self.phase_start_time = time.time()

                self.get_logger().info('Data ready. Settling...')
            return

        if self.phase == 'settling':
            self.publish_cmd()
            if time.time() - self.phase_start_time >= SETTLE_TIME:
                self.phase = 'driving'
                self.get_logger().info('Starting motion...')
            return

        if self.phase == 'driving':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= TARGET_DISTANCE:
                self.finish_and_exit()
                return

            self.publish_cmd(linear=self.linear_speed)


def main(args=None):
    rclpy.init(args=args)
    node = StraightnessTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()