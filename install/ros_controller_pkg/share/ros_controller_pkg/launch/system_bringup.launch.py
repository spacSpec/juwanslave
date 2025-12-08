from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1) PLC ↔ Modbus RTU 노드
        Node(
            package='plc_interface',
            executable='plc_node',      # ros2 run plc_interface plc_node
            name='plc_node',
            output='screen'
        ),

        # 2) 중앙 ros_controller 노드
        Node(
            package='ros_controller_pkg',
            executable='ros_controller.py',  # ros2 run ros_controller_pkg ros_controller.py
            name='ros_controller',
            output='screen'
        ),

        # 3) STM(ESP32) 제어 노드
        Node(
            package='stm_controller',
            executable='stm_node',      # ros2 run stm_controller stm_node
            name='stm_node',
            output='screen'
        ),
    ])
