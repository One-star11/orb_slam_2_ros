cmake_minimum_required(VERSION 3.8)
project(orb_slam2_ros)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check for c++14 support (ROS2 requirement)
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
IF(COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/orb_slam2/cmake_modules)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)

# ROS2 parameters configuration
configure_file(
  ${PROJECT_SOURCE_DIR}/ros/config/parameters.yaml
  ${PROJECT_SOURCE_DIR}/ros/config/parameters.yaml
  COPYONLY
)

set(LIBS_ORBSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/lib/libg2o.so
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/orb_slam2/lib/lib${PROJECT_NAME}.so
${OpenCV_LIBS}
)

# Add service files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)

include_directories(
${PROJECT_SOURCE_DIR}/orb_slam2
${PROJECT_SOURCE_DIR}/orb_slam2/include
${PROJECT_SOURCE_DIR}/ros/include
${EIGEN3_INCLUDE_DIR}
)

include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam2/lib)

add_library(${PROJECT_NAME} SHARED
orb_slam2/src/System.cc
orb_slam2/src/Tracking.cc
orb_slam2/src/LocalMapping.cc
orb_slam2/src/LoopClosing.cc
orb_slam2/src/ORBextractor.cc
orb_slam2/src/ORBmatcher.cc
orb_slam2/src/FrameDrawer.cc
orb_slam2/src/Converter.cc
orb_slam2/src/MapPoint.cc
orb_slam2/src/KeyFrame.cc
orb_slam2/src/Map.cc
orb_slam2/src/Optimizer.cc
orb_slam2/src/PnPsolver.cc
orb_slam2/src/Frame.cc
orb_slam2/src/KeyFrameDatabase.cc
orb_slam2/src/Sim3Solver.cc
orb_slam2/src/Initializer.cc
)
add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
${LIBS_ORBSLAM}
)

# map serialization addition - library boost serialization 
message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

# Mono node
add_executable (${PROJECT_NAME}_mono
ros/src/MonoNode.cc
ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_mono
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_srvs
  rcl_interfaces
)
rosidl_target_interfaces(${PROJECT_NAME}_mono ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_mono
${LIBS_ROS}
)

# Stereo node
add_executable (${PROJECT_NAME}_stereo
ros/src/StereoNode.cc
ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_stereo
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_srvs
  rcl_interfaces
)
rosidl_target_interfaces(${PROJECT_NAME}_stereo ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_stereo
${LIBS_ROS}
)

# RGBD node
add_executable (${PROJECT_NAME}_rgbd
ros/src/RGBDNode.cc
ros/src/Node.cc
)
ament_target_dependencies(${PROJECT_NAME}_rgbd
  rclcpp
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_geometry_msgs
  tf2_ros
  sensor_msgs
  geometry_msgs
  std_srvs
  rcl_interfaces
)
rosidl_target_interfaces(${PROJECT_NAME}_rgbd ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_rgbd
${LIBS_ROS}
)

# Install targets
install(TARGETS ${PROJECT_NAME}_mono ${PROJECT_NAME}_stereo ${PROJECT_NAME}_rgbd
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} DBoW2 g2o
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY ros/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY ros/config/
        DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY orb_slam2/config/
  DESTINATION share/${PROJECT_NAME}/orb_slam2/config
)

install(DIRECTORY orb_slam2/Vocabulary/
  DESTINATION share/${PROJECT_NAME}/orb_slam2/Vocabulary
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
