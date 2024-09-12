cmake_minimum_required(VERSION 3.8)
project(highbay_vicon_px4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  #${EIGEN3_INCLUDE_DIR}
)

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


# Declare libraries and executables
# add_library(State src/State.cpp)

add_executable(highbay_to_px4 src/highbayToPx4.cpp)

# Specify libraries to link a library or executable target against
# target_link_libraries(slung_pose_measurement
#   State
#   utils
#   frame_transforms
# )

# Add external dependencies
ament_target_dependencies(highbay_to_px4 rclcpp) #tf2_geometry_msgs tf2 Eigen3

# Install the libraries and executables
install(TARGETS
  highbay_to_px4
  DESTINATION lib/${PROJECT_NAME})


# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


# Install data (note without this get "unable to open file for logging" error)
# install(DIRECTORY
#     data
#     DESTINATION share/${PROJECT_NAME}/
# )


# Export dependencies
ament_package()