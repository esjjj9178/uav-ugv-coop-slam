cmake_minimum_required(VERSION 3.8)
project(couv_sim_robots)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY urdf meshes config launch rviz docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    couv_sim_robots/mission_runner.py
    couv_sim_robots/mission_aruco.py
    couv_sim_robots/gt_viz.py
    couv_sim_robots/record.sh
    couv_sim_robots/check_depth.py
    couv_sim_robots/topic_rates.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
