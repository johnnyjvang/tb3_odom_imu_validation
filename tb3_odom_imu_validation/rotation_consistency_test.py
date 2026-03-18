#!/usr/bin/env python3

"""
rotation_consistency_test.py

Rotate the robot in place and compare odometry yaw change against IMU yaw change.

Parameters:
- angular_speed (positive = CCW, negative = CW)
- test_name (used for JSON output)

Goal:
- Confirm odom and IMU report similar rotation
- Detect disagreement during in-place turning
- Save results to the JSON results file


Counter Clock Wise Test:
-------------------------------------------------------------------------
ros2 run tb3_odom_imu_validation rotation_consistency_test \
--ros-args -p angular_speed:=0.30 -p test_name:=rotation_consistency_ccw
-------------------------------------------------------------------------
Clock Wise Test:
-------------------------------------------------------------------------
ros2 run tb3_odom_imu_validation rotation_consistency_test \
--ros-args -p angular_speed:=-0.30 -p test_name:=rotation_consistency_cw
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


# ===== Default Test Settings =====
TARGET_ROTATION_DEG = 90.0
FAST_ZONE_DEG = 20.0
ANGULAR_SPEED_FAST = 0.30
ANGULAR_SPEED_SLOW = 0.10
ROTATION_STOP_TOL_DEG = 0.5

ODOM_IMU_DIFF_TOL_DEG = 5.0
SETTLE_TIME = 1.0


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RotationConsistencyTest(Node):
    def __init__(self):
        super().__init__('rotation_consistency_test')

        # ===== Parameters =====
        self.declare_parameter('angular_speed', ANGULAR_SPEED_FAST)
        self.declare_parameter('test_name', 'rotation_consistency_test')
        self.declare_parameter('target_rotation_deg', TARGET_ROTATION_DEG)

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.test_name = str(self.get_parameter('test_name').value)
        self.target_rotation_deg = float(self.get_parameter('target_rotation_deg').value)

        # ===== ROS interfaces =====
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        # ===== State =====
        self.current_odom_yaw = None
        self.current_imu_yaw = None

        self.start_odom_yaw = None
        self.start_imu_yaw = None

        self.phase = 'waiting_for_data'
        self.phase_start_time = time.time()
        self.done = False
        self.finish_time = None
        self.wait_log_time = 0.0
        self.last_progress_log_time = 0.0

        self.timer = self.create_timer(0.05, self.loop)

        direction = 'CCW' if self.angular_speed > 0 else 'CW'

        self.get_logger().info(f'Starting {self.test_name}')
        self.get_logger().info(f'Direction: {direction}')
        self.get_logger().info(f'Target rotation: {self.target_rotation_deg:.1f} deg')
        self.get_logger().info(f'Base angular speed: {self.angular_speed:.2f} rad/s')

    def odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.current_odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

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
            self.current_odom_yaw is not None and
            self.current_imu_yaw is not None
        )

    def signed_delta_deg(self, current, start):
        return math.degrees(normalize_angle(current - start))

    def abs_delta_deg(self, current, start):
        return abs(self.signed_delta_deg(current, start))

    def maybe_log_progress(self, odom_rot_deg, imu_rot_deg):
        now = time.time()
        if now - self.last_progress_log_time > 0.5:
            self.get_logger().info(
                f'[Progress] odom={odom_rot_deg:.1f}/{self.target_rotation_deg:.1f} deg | '
                f'imu={imu_rot_deg:.1f} deg | '
                f'diff={abs(imu_rot_deg - odom_rot_deg):.2f} deg'
            )
            self.last_progress_log_time = now

    def finish_and_exit(self):
        self.publish_cmd()
        time.sleep(0.1)
        self.publish_cmd()

        odom_rotation_deg = self.signed_delta_deg(self.current_odom_yaw, self.start_odom_yaw)
        imu_rotation_deg = self.signed_delta_deg(self.current_imu_yaw, self.start_imu_yaw)

        odom_rotation_abs_deg = abs(odom_rotation_deg)
        imu_rotation_abs_deg = abs(imu_rotation_deg)
        odom_imu_diff_deg = abs(imu_rotation_abs_deg - odom_rotation_abs_deg)

        target_ok = odom_rotation_abs_deg >= (self.target_rotation_deg - 2.0)
        consistency_ok = odom_imu_diff_deg <= ODOM_IMU_DIFF_TOL_DEG

        status = 'PASS' if (target_ok and consistency_ok) else 'FAIL'

        measurement = f'{odom_imu_diff_deg:.2f} deg'
        notes = (
            f'odom_rot={odom_rotation_abs_deg:.1f}deg, '
            f'imu_rot={imu_rotation_abs_deg:.1f}deg'
        )

        self.get_logger().info('=== Rotation Consistency Results ===')
        self.get_logger().info(f'Odom rotation: {odom_rotation_deg:.2f} deg')
        self.get_logger().info(f'IMU rotation: {imu_rotation_deg:.2f} deg')
        self.get_logger().info(f'Absolute odom rotation: {odom_rotation_abs_deg:.2f} deg')
        self.get_logger().info(f'Absolute IMU rotation: {imu_rotation_abs_deg:.2f} deg')
        self.get_logger().info(f'Odom vs IMU difference: {odom_imu_diff_deg:.2f} deg')
        self.get_logger().info(f'Result: {status}')

        append_result(
            test_name=self.test_name,
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info(f'Exiting {self.test_name}')
                rclpy.shutdown()
            return

        if self.phase == 'waiting_for_data':
            if self.data_ready():
                self.start_odom_yaw = self.current_odom_yaw
                self.start_imu_yaw = self.current_imu_yaw
                self.phase = 'settling'
                self.phase_start_time = time.time()

                self.get_logger().info('Received odom and IMU data')
                self.get_logger().info(
                    f'Start odom yaw: {math.degrees(self.start_odom_yaw):.2f} deg | '
                    f'Start imu yaw: {math.degrees(self.start_imu_yaw):.2f} deg'
                )
                self.get_logger().info(f'Settling for {SETTLE_TIME:.1f} second(s)...')
            else:
                now = time.time()
                if now - self.wait_log_time > 1.0:
                    missing = []
                    if self.current_odom_yaw is None:
                        missing.append('odom')
                    if self.current_imu_yaw is None:
                        missing.append('imu')
                    self.get_logger().info(f'Waiting for data: {missing}')
                    self.wait_log_time = now
            return

        if self.phase == 'settling':
            self.publish_cmd()
            if time.time() - self.phase_start_time >= SETTLE_TIME:
                self.phase = 'rotating'
                self.phase_start_time = time.time()
                self.get_logger().info('Starting rotation...')
            return

        if self.phase == 'rotating':
            odom_rot_deg = self.abs_delta_deg(self.current_odom_yaw, self.start_odom_yaw)
            imu_rot_deg = self.abs_delta_deg(self.current_imu_yaw, self.start_imu_yaw)

            self.maybe_log_progress(odom_rot_deg, imu_rot_deg)

            remaining_deg = self.target_rotation_deg - odom_rot_deg

            if remaining_deg <= ROTATION_STOP_TOL_DEG:
                self.get_logger().info('Target rotation reached')
                self.finish_and_exit()
                return

            commanded_speed = abs(self.angular_speed)
            if remaining_deg <= FAST_ZONE_DEG:
                commanded_speed = ANGULAR_SPEED_SLOW
            else:
                commanded_speed = min(abs(self.angular_speed), ANGULAR_SPEED_FAST)

            if self.angular_speed < 0.0:
                commanded_speed = -commanded_speed

            self.publish_cmd(angular=commanded_speed)


def main(args=None):
    rclpy.init(args=args)
    node = RotationConsistencyTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()