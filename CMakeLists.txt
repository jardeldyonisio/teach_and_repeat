cmake_minimum_required(VERSION 3.5)
project(teach_and_repeat)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

install(
  PROGRAMS
  src/utils.py
  src/copy_file.py
  src/compare_paths.py
  node/teach_path.py
  src/points_at_interval.py
  node/local_planner.py
  src/generate_lookahead.py
  src/save_coords_to_file.py
  src/generate_bezier_curve.py
  src/save_variables_to_file.py
  src/compare_bezier_lookahead.py
  src/create_folder_with_datetime.py
  DESTINATION lib/${PROJECT_NAME}
)

install( DIRECTORY rviz/
         DESTINATION share/${PROJECT_NAME}/rviz)
install( DIRECTORY launch/
         DESTINATION share/${PROJECT_NAME}/launch)
install( DIRECTORY path_saves/
         DESTINATION share/${PROJECT_NAME}/path_saves)
install( DIRECTORY node/
         DESTINATION share/${PROJECT_NAME}/node)
install( DIRECTORY param/
         DESTINATION share/${PROJECT_NAME}/param)
install( DIRECTORY src/
         DESTINATION share/${PROJECT_NAME}/src)
install( DIRECTORY bspline_smoother/
         DESTINATION share/${PROJECT_NAME}/bspline_smoother)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Make sure to export the generated interfaces
ament_export_dependencies(rosidl_default_runtime)
ament_package()