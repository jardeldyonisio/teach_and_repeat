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

install(
  PROGRAMS
  scripts/copy_file.py
  scripts/get_jacobian.py
  scripts/compare_paths.py
  scripts/compress_data.py
  scripts/calculate_erro.py
  nodes/teach_path_coords.py
  scripts/compare_paths_two.py
  scripts/points_at_interval.py
  nodes/repeat_path_coords.py
  scripts/generate_lookahead.py
  nodes/repeat_bezier_path.py
  scripts/save_coords_to_file.py
  scripts/generate_bezier_curve.py
  scripts/read_coords_from_file.py
  scripts/save_variables_to_file.py
  scripts/compare_bezier_lookahead.py
  scripts/create_folder_with_datetime.py
  src/PiecewiseG1BezierFit/scripts/pltC.py
  src/PiecewiseG1BezierFit/scripts/defk.py
  src/PiecewiseG1BezierFit/scripts/tang.py
  src/PiecewiseG1BezierFit/scripts/knots.py
  src/PiecewiseG1BezierFit/scripts/distEJL.py
  src/PiecewiseG1BezierFit/scripts/poplt.py
  src/PiecewiseG1BezierFit/scripts/ktangdt.py
  src/PiecewiseG1BezierFit/scripts/segop.py
  src/PiecewiseG1BezierFit/scripts/bstdst.py
  src/PiecewiseG1BezierFit/scripts/opdist.py
  src/PiecewiseG1BezierFit/scripts/NearestPoint.py
  src/PiecewiseG1BezierFit/scripts/findClosestPoint.py
  src/PiecewiseG1BezierFit/scripts/globop.py
  src/PiecewiseG1BezierFit/scripts/newk.py
  src/PiecewiseG1BezierFit/scripts/sod.py
  src/PiecewiseG1BezierFit/scripts/unitv.py
  src/PiecewiseG1BezierFit/scripts/ctpts.py
  src/PiecewiseG1BezierFit/scripts/cpoints.py
  src/PiecewiseG1BezierFit/scripts/DrawBezierCurve.py
  src/PiecewiseG1BezierFit/scripts/isAxisHandle.py
  src/PiecewiseG1BezierFit/scripts/drawPolyline.py
  src/PiecewiseG1BezierFit/scripts/objf2.py
  src/PiecewiseG1BezierFit/scripts/cubicBezierToPolyline.py
  src/PiecewiseG1BezierFit/scripts/iguess0.py
  src/PiecewiseG1BezierFit/scripts/BezierFitDemo.py
  DESTINATION lib/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install( DIRECTORY rviz/
         DESTINATION share/${PROJECT_NAME}/rviz)
install( DIRECTORY demonstrated_path/
         DESTINATION share/${PROJECT_NAME}/demonstrated_path)
install( DIRECTORY data/
         DESTINATION share/${PROJECT_NAME}/data) 

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
