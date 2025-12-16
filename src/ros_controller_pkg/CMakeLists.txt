cmake_minimum_required(VERSION 3.5)
project(ros_controller_pkg)

# 기본 ament_cmake
find_package(ament_cmake REQUIRED)

# ROS2 의존성
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ─────────────────────────────────────────────
# 1) msg 빌드 (PlcStatus.msg)
# ─────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlcStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# ─────────────────────────────────────────────
# 2) 파이썬 노드 설치
#    ros2 run ros_controller_pkg ros_controller.py
#    → install/.../lib/ros_controller_pkg/ros_controller.py 실행
# ─────────────────────────────────────────────
install(
  PROGRAMS
    ros_controller_pkg/ros_controller.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  ros_controller_pkg/db_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

# ─────────────────────────────────────────────
# 3) launch 파일 설치
# ─────────────────────────────────────────────
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
