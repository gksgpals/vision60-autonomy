cmake_minimum_required(VERSION 3.8)
project(vision60_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(operator_config_test test/test_operator_config.py)
endif()

ament_package()
