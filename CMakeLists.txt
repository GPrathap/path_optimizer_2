cmake_minimum_required(VERSION 3.5)
project(path_optimizer_2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_viz_tools REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tinyspline_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(gflags REQUIRED)


set(DEPENDENCIES
  "rclcpp"
  "ros2_viz_tools"
  "grid_map_ros"
  "tinyspline_ros"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
)


include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
        src/tools/tools.cpp
        src/tools/spline.cpp
        src/path_optimizer.cpp
        src/tools/collision_checker.cpp
        src/reference_path_smoother/reference_path_smoother.cpp
        src/tools/Map.cpp
        src/tools/car_geometry.cpp
        src/solver/base_solver.cpp
        src/data_struct/date_struct.cpp
        src/data_struct/reference_path_impl.cpp
        src/data_struct/reference_path.cpp
        src/data_struct/vehicle_state_frenet.cpp
        src/config/planning_flags.cpp
        src/reference_path_smoother/tension_smoother.cpp
        src/reference_path_smoother/tension_smoother_2.cpp
        src/tools/time_recorder.cpp)

target_link_libraries(${PROJECT_NAME} glog gflags OsqpEigen::OsqpEigen osqp::osqp)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

add_executable(${PROJECT_NAME}_demo src/test/demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_demo ${DEPENDENCIES})

# install(TARGETS ${PROJECT_NAME}_demo ${PROJECT_NAME}
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch cmake script 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${DEPENDENCIES})
ament_package()
