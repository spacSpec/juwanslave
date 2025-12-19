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
            executable='ros_controller.py',
            name='ros_controller',
            output='screen'
        ),

        # 3) STM(ESP32) 제어 노드
        Node(
            package='stm_controller',
            executable='stm_node',
            name='stm_node',
            output='screen'
        ),

        # 4) PLC DB 로거 (db.py)
        Node(
            package='plc_interface',
            executable='db',        # ★ 방금 setup.py 에 등록한 이름
            name='plc_db_logger',
            output='screen'
        ),
    ])
