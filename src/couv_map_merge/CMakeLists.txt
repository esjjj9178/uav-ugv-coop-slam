cmake_minimum_required(VERSION 3.8)
project(couv_map_merge)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch rviz docs DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS couv_map_merge/run_slam.sh couv_map_merge/merge_viz.py couv_map_merge/run_live.sh couv_map_merge/aruco_detector.py couv_map_merge/compare_aruco.py couv_map_merge/watch_live.sh
    couv_map_merge/aruco_viz.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
