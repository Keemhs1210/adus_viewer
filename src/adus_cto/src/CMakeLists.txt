cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)

project(adus_cto)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
# SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  adss_msgs
  morai_msgs

)

catkin_package(
  CATKIN_DEPENDS 
  rospy
  std_msgs
  adss_msgs
  morai_msgs
)

include_directories(/usr/include)
include_directories(${PROJECT_SOURCE_DIR}/include/adus_cto)
include_directories(
	${catkin_INCLUDE_DIRS}
  include
)

add_executable(path_read_node src/path_read.cpp src/path_read_node.cpp)
target_include_directories(path_read_node PRIVATE)
target_link_libraries(path_read_node ${catkin_LIBRARIES})

add_executable(key_input_node src/key_input.cpp src/key_input_node.cpp)
target_include_directories(key_input_node PRIVATE)
target_link_libraries(key_input_node ${catkin_LIBRARIES})

add_executable(path_maker_kkyu src/path_maker_node_kkyu.cpp src/path_maker_kkyu.cpp src/func_kkyu.cpp src/interpolation_kkyu.cpp src/utm_transfer.cpp)
target_include_directories(path_maker_kkyu PRIVATE)
target_link_libraries(path_maker_kkyu ${catkin_LIBRARIES})


install(DIRECTORY
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS 
path_read_node
key_input_node
path_maker_kkyu
#${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
