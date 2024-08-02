cmake_minimum_required(VERSION 3.5)
project(freedom_navigation)

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

install(
  PROGRAMS
  scripts/copy_file.py
  scripts/plot_bezier.py
  scripts/get_jacobian.py
  scripts/compare_paths.py
  scripts/compress_data.py
  scripts/calculate_erro.py
  scripts/view_path_coords.py
  scripts/teach_path_coords.py
  scripts/compare_paths_two.py
  scripts/points_at_interval.py
  scripts/repeat_path_coords.py
  scripts/generate_lookahead.py
  scripts/repeat_bezier_path.py
  scripts/save_coords_to_file.py
  scripts/generate_bezier_curve.py
  scripts/read_coords_from_file.py
  scripts/save_variables_to_file.py
  scripts/compare_bezier_lookahead.py
  scripts/create_folder_with_datetime.py
  src/PiecewiseG1BezierFitPython/scripts/pltC.py
  src/PiecewiseG1BezierFitPython/scripts/defk.py
  src/PiecewiseG1BezierFitPython/scripts/tang.py
  src/PiecewiseG1BezierFitPython/scripts/knots.py
  src/PiecewiseG1BezierFitPython/scripts/distEJL.py
  src/PiecewiseG1BezierFitPython/scripts/iguess0.py
  src/PiecewiseG1BezierFitPython/scripts/BezierFitDemo.py
  DESTINATION lib/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install( DIRECTORY launch/
         DESTINATION share/${PROJECT_NAME}/launch) 
install( DIRECTORY config/
         DESTINATION share/${PROJECT_NAME}/config) 

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

ament_package()
