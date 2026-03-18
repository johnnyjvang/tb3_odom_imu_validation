"""
odom_imu_validation_all.launch.py

Runs TurtleBot3 odom + IMU validation tests sequentially
and prints a summary report at the end.
"""

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    # Reset results file before starting tests
    reset_results = Node(
        package='tb3_odom_imu_validation',
        executable='reset_results',
        output='screen'
    )

    # Test 1: Forward straightness
    forward_straightness = Node(
        package='tb3_odom_imu_validation',
        executable='straightness_test',
        output='screen',
        parameters=[
            {'linear_speed': 0.08},
            {'test_name': 'forward_straightness'}
        ]
    )

    # Test 2: Backward straightness
    backward_straightness = Node(
        package='tb3_odom_imu_validation',
        executable='straightness_test',
        output='screen',
        parameters=[
            {'linear_speed': -0.08},
            {'test_name': 'backward_straightness'}
        ]
    )

    # Test 3: Rotation consistency CCW
    rotation_consistency_ccw = Node(
        package='tb3_odom_imu_validation',
        executable='rotation_consistency_test',
        output='screen',
        parameters=[
            {'angular_speed': 0.30},
            {'test_name': 'rotation_consistency_ccw'},
            {'target_rotation_deg': 90.0}
        ]
    )

    # Test 4: Rotation consistency CW
    rotation_consistency_cw = Node(
        package='tb3_odom_imu_validation',
        executable='rotation_consistency_test',
        output='screen',
        parameters=[
            {'angular_speed': -0.30},
            {'test_name': 'rotation_consistency_cw'},
            {'target_rotation_deg': 90.0}
        ]
    )

    # Test 5: Out and back heading
    out_and_back_heading = Node(
        package='tb3_odom_imu_validation',
        executable='out_and_back_heading',
        output='screen'
    )

    # Final summary report
    summary_report = Node(
        package='tb3_odom_imu_validation',
        executable='summary_report',
        output='screen'
    )

    return LaunchDescription([

        # Start by resetting results file
        reset_results,

        # When reset_results exits, start forward_straightness
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_results,
                on_exit=[TimerAction(period=1.0, actions=[forward_straightness])]
            )
        ),

        # When forward_straightness exits, start backward_straightness
        RegisterEventHandler(
            OnProcessExit(
                target_action=forward_straightness,
                on_exit=[TimerAction(period=1.0, actions=[backward_straightness])]
            )
        ),

        # When backward_straightness exits, start rotation_consistency_ccw
        RegisterEventHandler(
            OnProcessExit(
                target_action=backward_straightness,
                on_exit=[TimerAction(period=1.0, actions=[rotation_consistency_ccw])]
            )
        ),

        # When rotation_consistency_ccw exits, start rotation_consistency_cw
        RegisterEventHandler(
            OnProcessExit(
                target_action=rotation_consistency_ccw,
                on_exit=[TimerAction(period=1.0, actions=[rotation_consistency_cw])]
            )
        ),

        # When rotation_consistency_cw exits, start out_and_back_heading
        RegisterEventHandler(
            OnProcessExit(
                target_action=rotation_consistency_cw,
                on_exit=[TimerAction(period=1.0, actions=[out_and_back_heading])]
            )
        ),

        # When out_and_back_heading exits, start summary_report
        RegisterEventHandler(
            OnProcessExit(
                target_action=out_and_back_heading,
                on_exit=[TimerAction(period=1.0, actions=[summary_report])]
            )
        ),

    ])