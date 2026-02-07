
from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    # กำหนดค่าพารามิเตอร์ตรงนี้เลย (ไม่ต้องง้อไฟล์ YAML)

    f1tenth_config = {

        'port': '/dev/ttyACM0',

        'baudrate': 115200,

  # ค่าจูน VESC

        'speed_to_erpm_gain': 4614.0,

        'speed_to_erpm_offset': 0.0,

        'steering_angle_to_servo_gain': -1.213,

        'steering_angle_to_servo_offset': 0.5304,


        # * ปลดล็อคขีดจำกัด (สำคัญมาก!) *

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

        Node(

            package='vesc_driver',

            executable='vesc_driver_node',

            name='vesc_driver_node',

            output='screen',

            parameters=[f1tenth_config]

        ),

        Node(

            package='vesc_ackermann',

            executable='ackermann_to_vesc_node',

            name='ackermann_to_vesc_node',

            output='screen',

            parameters=[f1tenth_config],

            remappings=[

                ('ackermann_cmd', 'drive')

            ]

        )

    ])

