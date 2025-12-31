cmake_minimum_required(VERSION 3.8)
project(turtlebot3_wall_following_node)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(turtlebot3_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Find all C++ source and header files
file(GLOB_RECURSE ALL_CXX_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
)

# Create a format target that runs automatically
find_program(CLANG_FORMAT "clang-format")
if(CLANG_FORMAT)
  add_custom_target(
    format_code ALL
    COMMAND ${CLANG_FORMAT} -i ${ALL_CXX_SOURCE_FILES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
  )
endif()

set(SOURCES
  src/msg_utils.cpp
  src/turtlebot3_params.cpp
  src/wall_following_controller.cpp
)
set(LIBRARY spiral_turtlebot3_library)
add_library(${LIBRARY} STATIC ${SOURCES})

# Ensure formatting happens before library compilation
if(CLANG_FORMAT)
  add_dependencies(${LIBRARY} format_code)
endif()

target_include_directories(${LIBRARY} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${LIBRARY} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ${LIBRARY}
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "tf2_eigen"
  "turtlebot3_msgs"
  "sensor_msgs"
  "nav_msgs"
  "geometry_msgs"
  "rclcpp"
  "visualization_msgs"
  "rosbag2_cpp"
)
target_link_libraries(${LIBRARY} Eigen3::Eigen)

set(WALL_FOLLOWING_NODE turtlebot3_wall_following_node)
add_executable(${WALL_FOLLOWING_NODE} "src/${WALL_FOLLOWING_NODE}.cpp")
target_link_libraries(${WALL_FOLLOWING_NODE} PRIVATE ${LIBRARY})

install(TARGETS
  ${LIBRARY}
  ${WALL_FOLLOWING_NODE}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp tf2 tf2_ros tf2_geometry_msgs tf2_eigen
  turtlebot3_msgs sensor_msgs nav_msgs geometry_msgs visualization_msgs
  rosbag2_cpp
  Eigen3
)

set(TEST_SOURCES
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
  if(TEST_SOURCES)
    find_package(ament_cmake_gtest REQUIRED)
    set(TEST_EXECUTABLE ${PROJECT_NAME}_test)
    ament_add_gtest(${TEST_EXECUTABLE} ${TEST_SOURCES})
    target_include_directories(${TEST_EXECUTABLE} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${TEST_EXECUTABLE} ${LIBRARY})
  endif()
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
