cmake_minimum_required(VERSION 3.8)
project(gui_arm_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(brics_actuator REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 2. Включаем автоматическую обработку мета-объектов и UI-файлов
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp
  src/mainwindow.cpp
  src/iksolver.cpp
  include/mainwindow.h
  include/iksolver.h
  ui/mainwindow.ui
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${orocos_kdl_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp sensor_msgs control_msgs rclcpp_action brics_actuator urdf          # Добавили
  kdl_parser    # Добавили
  orocos_kdl    # Добавили
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})
ament_package()
