cmake_minimum_required(VERSION 3.5)
project(diff_car)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  diff_car/demo_simple_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME demo_simple_navigator
)

install(PROGRAMS
  diff_car/astar_navigator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME astar_navigator
)

install(PROGRAMS
  diff_car/keyboard_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_control
)

install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
