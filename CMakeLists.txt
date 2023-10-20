cmake_minimum_required(VERSION 3.11)
project(detect_balloon VERSION 1.0.3 LANGUAGES CXX)

# set C++ standard
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  px4_cmd
  std_msgs
  mavros_msgs
  geometry_msgs
  message_generation
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cv_bridge px4_cmd image_transport roscpp sensors_msgs std_msgs message_runtime geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(detect_balloon src/detect_balloon.cpp)
add_dependencies(detect_balloon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(opencv_test_node opencv_test_node_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(detect_balloon ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
