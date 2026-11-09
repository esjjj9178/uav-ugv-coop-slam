cmake_minimum_required(VERSION 3.8)
project(couv_sim_world)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY worlds models launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gen_textures.py scripts/gen_world.py scripts/capture_views.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  FILES scripts/layout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
