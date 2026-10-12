cmake_minimum_required(VERSION 3.8)
project(spot_micro_camera)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# spot_micro_camera.detect: which camera is plugged in (used by camera.launch.py
# and spot_micro_deck's bridge).
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_detect test/test_detect.py)
endif()

ament_package()
