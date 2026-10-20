cmake_minimum_required(VERSION 3.8)
project(ur5_moveit_toolkit)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python API: `from ur5_moveit_toolkit import UR5`
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config urdf poses
  DESTINATION share/${PROJECT_NAME}
)

# Command-line tools: `ros2 run ur5_moveit_toolkit <name>`
file(GLOB TOOLKIT_SCRIPTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*)
install(PROGRAMS ${TOOLKIT_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
