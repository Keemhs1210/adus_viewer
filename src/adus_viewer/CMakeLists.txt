
# ------------------------------------------------
# Project Setup
# ------------------------------------------------
# Specify minimum CMake version required for this project
cmake_minimum_required(VERSION 3.9)

# ------------------------------------------------
# Set default C and C++ language standards if not already defined
# ------------------------------------------------
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# ------------------------------------------------
# Project Name
# ------------------------------------------------
project(adus_viewer)

# ------------------------------------------------
# Compiler-specific options
# ------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# ------------------------------------------------
# Find catkin and required packages
# ------------------------------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rviz
  cv_bridge
  std_msgs
  sensor_msgs
  adss_msgs
  morai_msgs
  jsk_rviz_plugins
  visualization_msgs
)
# ------------------------------------------------
# Declare catkin package
# ------------------------------------------------
catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  adss_msgs
  morai_msgs
  visualization_msgs
)

# ------------------------------------------------
# Add executable
# ------------------------------------------------
add_executable(${PROJECT_NAME}_node
  src/Viewer/viewer_node.cpp
  src/Viewer/viewer.cpp
  src/Input/input_topic.cpp
  src/Logic/preprocess.cpp
)

# ------------------------------------------------
# Include directories
# ------------------------------------------------
include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${std_msgs_INCLUDE_DIRS}
)

# ------------------------------------------------
# Link libraries to the executable
# ------------------------------------------------
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  src/Viewer
  src/Input
  src/Logic
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES} 
  ${roscpp_LIBRARIES}
  ${roslib_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${sensor_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
# ------------------------------------------------
# Install configuration
# ------------------------------------------------
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)