cmake_minimum_required(VERSION 3.8)
project(car_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
include_directories(include ${QT5Widgets_INCLUDE_DIRS})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
qt5_wrap_cpp(header_SRC include/car_gui/mainwindow.h)

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

add_executable(car_gui 
    ${header_SRC}    
    src/car_gui.cpp
    src/mainwindow.cpp)

target_link_libraries(car_gui Qt5::Widgets)

ament_target_dependencies(car_gui rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)

include_directories(car_gui
    include/car_gui/
    ${CMAKE_CURRENT_BINARY_DIR}
)

install(TARGETS car_gui
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()
