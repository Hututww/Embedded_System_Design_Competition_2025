cmake_minimum_required(VERSION 3.8)
project(fire_drone)

# 配置 C/C++ 标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 查找ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 新增：消息生成依赖
find_package(rosidl_default_generators REQUIRED)

# 新增：定义消息文件
set(msg_files
  "msg/FireDetectionResult.msg"
)

# 新增：生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs geometry_msgs
)

# 安装Python模块和脚本
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  src/yolov5_detector.py
  src/drone_controller.py
  src/main_controller.py
  src/ir_camera.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件和模型权重
install(DIRECTORY
  config
  launch
  weights
  yolo
  DESTINATION share/${PROJECT_NAME}
)

# 新增：确保消息生成后再编译依赖包
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 新增：导出消息依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()