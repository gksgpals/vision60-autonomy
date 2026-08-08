cmake_minimum_required(VERSION 3.8)
project(vision60_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/digital_twin_probe
  scripts/digital_twin_obstacle_probe
  scripts/digital_twin_motion_probe
  scripts/digital_twin_recovery_harness
  scripts/digital_twin_integrated_recovery_harness
  scripts/digital_twin_frontier_harness
  scripts/digital_twin_perception_harness
  scripts/pose_to_odometry
  scripts/body_twist_to_world
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(simulation_assets_test test/test_simulation_assets.py)
endif()

ament_package()
