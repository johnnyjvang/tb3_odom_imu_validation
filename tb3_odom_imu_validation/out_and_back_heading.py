#!/usr/bin/env python3

"""
out_and_back_heading.py

Drive forward 1 meter, rotate 180 degrees, then drive forward 1 meter
back toward the starting point.

Measure:
- final closure error relative to the start
- total heading change from odom
- total heading change from IMU
- final heading agreement between odom and IMU

Goal:
- Evaluate combined translation + rotation accuracy
- Detect accumulated heading disagreement
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

TARGET_DISTANCE = 1.0
TARGET_ROTATION_DEG = 180.0
ROTATION_STOP_TOLERANCE_DEG = 0.5

LINEAR_SPEED = 0.10
ANGULAR_SPEED_FAST = 0.30
ANGULAR_SPEED_SLOW = 0.10

CLOSURE_TOL = 0.10
HEADING_DIFF_TOL_DEG = 8.0

SETTLE_TIME = 1.0


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class OutAndBackHeading(Node):
    def __init__(self):
        super().__init__('out_and_back_heading')

        self.pub = self.create_publisher(TwistStamped, CMD_VEL_TOPIC, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.imu_sub = self.create_subscription(Imu, IMU_TOPIC, self.imu_cb, 10)

        # Start pose
        self.start_x = None
        self.start_y = None
        self.start_odom_yaw = None
        self.start_imu_yaw = None

        # Current pose
        self.current_x = None
        self.current_y = None
        self.current_odom_yaw = None
        self.current_imu_yaw = None

        # Per-phase start pose
        self.phase_start_x = None
        self.phase_start_y = None
        self.phase_start_odom_yaw = None
        self.phase_start_imu_yaw = None

        # Measurements
        self.forward_distance = 0.0
        self.rotation_odom_deg = 0.0
        self.rotation_imu_deg = 0.0
        self.return_distance = 0.0

        # State
        self.phase = 'waiting_for_data'
        self.phase_start_time = time.time()
        self.last_progress_log_time = 0.0
        self.wait_log_time = 0.0

        self.done = False
        self.finish_time = None

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('Starting out_and_back_heading test')
        self.get_logger().info(f'Target distance: {TARGET_DISTANCE:.2f} m each leg')
        self.get_logger().info(f'Target rotation: {TARGET_ROTATION_DEG:.1f} deg')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def imu_cb(self, msg):
        q = msg.orientation
        self.current_imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def data_ready(self):
        return (
            self.current_x is not None and
            self.current_y is not None and
            self.current_odom_yaw is not None and
            self.current_imu_yaw is not None
        )

    def publish(self, x=0.0, z=0.0):
        msg = TwistStamped()
        msg.twist.linear.x = x
        msg.twist.angular.z = z
        self.pub.publish(msg)

    def set_new_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = time.time()
        self.phase_start_x = self.current_x
        self.phase_start_y = self.current_y
        self.phase_start_odom_yaw = self.current_odom_yaw
        self.phase_start_imu_yaw = self.current_imu_yaw
        self.last_progress_log_time = 0.0
        self.get_logger().info(f'Starting phase: {new_phase}')

    def maybe_log_progress(self, message):
        now = time.time()
        if now - self.last_progress_log_time > 0.5:
            self.get_logger().info(message)
            self.last_progress_log_time = now

    def finish_and_exit(self):
        self.publish()
        time.sleep(0.1)
        self.publish()

        final_error = math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

        total_odom_heading_deg = math.degrees(
            normalize_angle(self.current_odom_yaw - self.start_odom_yaw)
        )
        total_imu_heading_deg = math.degrees(
            normalize_angle(self.current_imu_yaw - self.start_imu_yaw)
        )

        final_heading_diff_deg = abs(
            math.degrees(
                normalize_angle(self.current_imu_yaw - self.current_odom_yaw)
            )
        )

        closure_ok = final_error <= CLOSURE_TOL
        heading_ok = final_heading_diff_deg <= HEADING_DIFF_TOL_DEG

        status = 'PASS' if (closure_ok and heading_ok) else 'FAIL'
        measurement = f'{final_error:.3f} m'
        notes = (
            f'final_heading_diff={final_heading_diff_deg:.2f}deg, '
            f'odom_heading={total_odom_heading_deg:.2f}deg, '
            f'imu_heading={total_imu_heading_deg:.2f}deg'
        )

        self.get_logger().info('===== Out and Back Heading Results =====')
        self.get_logger().info(f'Forward distance traveled: {self.forward_distance:.3f} m')
        self.get_logger().info(f'Rotation traveled (odom): {self.rotation_odom_deg:.2f} deg')
        self.get_logger().info(f'Rotation traveled (imu): {self.rotation_imu_deg:.2f} deg')
        self.get_logger().info(f'Return distance traveled: {self.return_distance:.3f} m')
        self.get_logger().info(f'Final closure error: {final_error:.3f} m')
        self.get_logger().info(f'Total odom heading change: {total_odom_heading_deg:.2f} deg')
        self.get_logger().info(f'Total IMU heading change: {total_imu_heading_deg:.2f} deg')
        self.get_logger().info(f'Final odom vs IMU heading difference: {final_heading_diff_deg:.2f} deg')
        self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='out_and_back_heading',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting out_and_back_heading')
                rclpy.shutdown()
            return

        if self.phase == 'waiting_for_data':
            if self.data_ready():
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.start_odom_yaw = self.current_odom_yaw
                self.start_imu_yaw = self.current_imu_yaw

                self.phase_start_x = self.current_x
                self.phase_start_y = self.current_y
                self.phase_start_odom_yaw = self.current_odom_yaw
                self.phase_start_imu_yaw = self.current_imu_yaw

                self.phase = 'settling'
                self.phase_start_time = time.time()

                self.get_logger().info('Captured starting pose')
                self.get_logger().info(
                    f'Start -> x: {self.start_x:.3f}, y: {self.start_y:.3f}, '
                    f'odom yaw: {math.degrees(self.start_odom_yaw):.2f} deg, '
                    f'imu yaw: {math.degrees(self.start_imu_yaw):.2f} deg'
                )
                self.get_logger().info(f'Settling for {SETTLE_TIME:.1f} second(s)...')
            else:
                now = time.time()
                if now - self.wait_log_time > 1.0:
                    missing = []
                    if self.current_x is None or self.current_y is None or self.current_odom_yaw is None:
                        missing.append('odom')
                    if self.current_imu_yaw is None:
                        missing.append('imu')
                    self.get_logger().info(f'Waiting for data: {missing}')
                    self.wait_log_time = now
            return

        if self.phase == 'settling':
            self.publish()
            if time.time() - self.phase_start_time >= SETTLE_TIME:
                self.set_new_phase('forward_1')
            return

        if self.phase == 'forward_1':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Forward leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.forward_distance = dist
                self.get_logger().info(
                    f'Forward leg complete. Distance traveled: {dist:.3f} m'
                )
                self.set_new_phase('rotate_180')

        elif self.phase == 'rotate_180':
            odom_delta_deg = abs(math.degrees(
                normalize_angle(self.current_odom_yaw - self.phase_start_odom_yaw)
            ))
            imu_delta_deg = abs(math.degrees(
                normalize_angle(self.current_imu_yaw - self.phase_start_imu_yaw)
            ))

            self.maybe_log_progress(
                f'Rotation progress: odom={odom_delta_deg:.1f} deg / {TARGET_ROTATION_DEG:.1f} deg | '
                f'imu={imu_delta_deg:.1f} deg'
            )

            remaining_error = TARGET_ROTATION_DEG - odom_delta_deg

            if remaining_error > 20.0:
                self.publish(z=ANGULAR_SPEED_FAST)
            elif remaining_error > ROTATION_STOP_TOLERANCE_DEG:
                self.publish(z=ANGULAR_SPEED_SLOW)
            else:
                self.publish()
                self.rotation_odom_deg = odom_delta_deg
                self.rotation_imu_deg = imu_delta_deg
                self.get_logger().info(
                    f'Rotation complete. Odom: {odom_delta_deg:.1f} deg | IMU: {imu_delta_deg:.1f} deg'
                )
                self.set_new_phase('forward_2')

        elif self.phase == 'forward_2':
            dist = math.sqrt(
                (self.current_x - self.phase_start_x) ** 2 +
                (self.current_y - self.phase_start_y) ** 2
            )

            self.maybe_log_progress(
                f'Return leg distance: {dist:.3f} m / {TARGET_DISTANCE:.3f} m'
            )

            if dist < TARGET_DISTANCE:
                self.publish(x=LINEAR_SPEED)
            else:
                self.publish()
                self.return_distance = dist
                self.get_logger().info(
                    f'Return leg complete. Distance traveled: {dist:.3f} m'
                )
                self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = OutAndBackHeading()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()