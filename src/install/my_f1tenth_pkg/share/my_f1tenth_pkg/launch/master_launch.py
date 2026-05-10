from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    f1tenth_config = {
        'port': '/dev/ttyACM0',
        'baudrate': 115200,
        'speed_to_erpm_gain': 3747.1,
        'speed_to_erpm_offset': 0.0,
        'steering_angle_to_servo_gain': -1.213,
        'steering_angle_to_servo_offset': 0.5,
        'servo_min': 0.15,
        'servo_max': 0.85,
        'speed_min': -10000.0,
        'speed_max': 10000.0,
        'duty_cycle_min': 0.0,
        'duty_cycle_max': 0.95,
        'current_min': 0.0,
        'current_max': 100.0
    }

    return LaunchDescription([
        # 1. VESC Driver & Ackermann
        Node(package='vesc_driver', executable='vesc_driver_node', parameters=[f1tenth_config]),
        Node(package='vesc_ackermann', executable='ackermann_to_vesc_node', parameters=[f1tenth_config], remappings=[('ackermann_cmd', 'drive')]),

        # 2. Odometry Nodes
        Node(package='my_f1tenth_pkg', executable='newodom_node', name='newodom'),
        Node(package='my_f1tenth_pkg', executable='odom_erpm_node', name='odom_erpm', remappings=[('/path', '/path/erpm')]),
        Node(package='my_f1tenth_pkg', executable='odom_tacho_node', name='odom_tacho', remappings=[('/path', '/path/tacho')]),
    ])
