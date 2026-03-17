#!/usr/bin/env python3

"""
forward_straightness.py

Drive the robot forward using odometry distance and evaluate how straight
the motion was using:
- odometry lateral drift
- IMU heading change during the forward motion

Goal:
- Confirm the robot can drive forward in a mostly straight line
- Detect heading drift while translating
- Save results to the JSON results file
"""

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tb3_odom_imu_validation.result_utils import append_result


# ===== Test Settings =====
CMD_VEL_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'
IMU_TOPIC = '/imu'

TARGET_DISTANCE = 1.0          # meters
LINEAR_SPEED = 0.08            # m/s

LATERAL_DRIFT_TOL = 0.05       # meters
HEADING_DRIFT_TOL_DEG = 5.0    # degrees

SETTLE_TIME = 1.0              # seconds before motion starts


def quaternion_to_yaw(qx, qy, qz, qw):
    """Convert quaternion to yaw in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ForwardStraightness(Node):
    def __init__(self):
        super().__init__('forward_straightness')

        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)

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

        self.timer = self.create_timer(0.05, self.loop)
        self.progress_timer = self.create_timer(0.5, self.progress_update)
        self.wait_log_time = 0.0

        self.get_logger().info('Starting forward_straightness test')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.2f} m')
        self.get_logger().info(f'Linear speed: {LINEAR_SPEED:.2f} m/s')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def publish_stop(self):
        msg = TwistStamped()
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def publish_forward(self):
        msg = TwistStamped()
        msg.twist.linear.x = LINEAR_SPEED
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def data_ready(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_imu_yaw is not None
        )

    def progress_update(self):
        if self.done:
            return

        if self.phase == 'driving' and self.start_x is not None and self.start_y is not None:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            heading_change_deg = math.degrees(
                normalize_angle(self.current_imu_yaw - self.start_imu_yaw)
            )

            self.get_logger().info(
                f'[Progress] distance={distance:.3f} m / {TARGET_DISTANCE:.3f} m | '
                f'lateral drift={dy:.3f} m | '
                f'imu heading drift={heading_change_deg:.2f} deg'
            )

    def finish_and_exit(self):
        self.publish_stop()
        time.sleep(0.1)
        self.publish_stop()

        if self.start_x is None or self.start_y is None or self.start_imu_yaw is None:
            status = 'FAIL'
            measurement = '0.00 m'
            notes = 'Test did not initialize start state correctly'
            self.get_logger().error('Test failed: missing start state')
        else:
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            lateral_drift = dy
            heading_change_rad = normalize_angle(self.current_imu_yaw - self.start_imu_yaw)
            heading_change_deg = math.degrees(heading_change_rad)

            lateral_ok = abs(lateral_drift) <= LATERAL_DRIFT_TOL
            heading_ok = abs(heading_change_deg) <= HEADING_DRIFT_TOL_DEG
            distance_ok = distance >= (TARGET_DISTANCE * 0.98)

            status = 'PASS' if (lateral_ok and heading_ok and distance_ok) else 'FAIL'
            measurement = f'{distance:.3f} m'
            notes = (
                f'lateral_drift={lateral_drift:.3f}m, '
                f'imu_heading_change={heading_change_deg:.2f}deg'
            )

            self.get_logger().info('=== Forward Straightness Results ===')
            self.get_logger().info(f'Forward distance traveled: {distance:.3f} m')
            self.get_logger().info(f'X displacement: {dx:.3f} m')
            self.get_logger().info(f'Y displacement (lateral drift): {dy:.3f} m')
            self.get_logger().info(f'IMU heading change: {heading_change_deg:.2f} deg')
            self.get_logger().info(f'Lateral drift tolerance: +/- {LATERAL_DRIFT_TOL:.3f} m')
            self.get_logger().info(f'Heading drift tolerance: +/- {HEADING_DRIFT_TOL_DEG:.2f} deg')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='forward_straightness',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting forward_straightness')
                rclpy.shutdown()
            return

        if self.phase == 'waiting_for_data':
            if self.data_ready():
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_imu_yaw = self.current_imu_yaw
                self.phase = 'settling'
                self.phase_start_time = time.time()

                self.get_logger().info('Received odom and IMU data')
                self.get_logger().info(
                    f'Start pose -> x: {self.start_x:.3f}, y: {self.start_y:.3f}, '
                    f'imu yaw: {math.degrees(self.start_imu_yaw):.2f} deg'
                )
                self.get_logger().info(f'Settling for {SETTLE_TIME:.1f} second(s)...')
            else:
                now = time.time()
                if now - self.wait_log_time > 1.0:
                    missing = []
                    if self.current_x is None or self.current_y is None:
                        missing.append('odom')
                    if self.current_imu_yaw is None:
                        missing.append('imu')
                    self.get_logger().info(f'Waiting for data: {missing}')
                    self.wait_log_time = now
            return

        if self.phase == 'settling':
            self.publish_stop()
            if time.time() - self.phase_start_time >= SETTLE_TIME:
                self.phase = 'driving'
                self.phase_start_time = time.time()
                self.get_logger().info('Starting forward motion...')
            return

        if self.phase == 'driving':
            dx = self.current_x - self.start_x
            dy = self.current_y - self.start_y
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= TARGET_DISTANCE:
                self.get_logger().info('Target forward distance reached')
                self.phase = 'finished'
                self.finish_and_exit()
                return

            self.publish_forward()


def main(args=None):
    rclpy.init(args=args)
    node = ForwardStraightness()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()