cmake_minimum_required(VERSION 3.5)
project(cubeeye_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
add_compile_options(-Wno-unused-parameter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LastState.srv"
  "srv/LastError.srv"
  "srv/Scan.srv"
  "srv/Connect.srv"
  "srv/Run.srv"
  "srv/Stop.srv"
  "srv/Disconnect.srv"
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/include/CubeEye
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/lib
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release
  ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib
)

add_executable(${PROJECT_NAME}_node 
  src/CameraModule.cpp
  src/CubeEyeCameraNode.cpp
  src/ModelParameter.cpp
  src/SCubeModelParameter.cpp
  src/ICubeModelParameter.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(${PROJECT_NAME}_node
   rclcpp
   rclcpp_action
   cv_bridge
   std_srvs
   std_msgs
   sensor_msgs
   geometry_msgs
   tf2
   tf2_ros
   tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  "${cpp_typesupport_target}"
  CubeEye
  ${OpenCV_LIBRARIES}
)

# Use old RPATH. 
# https://stackoverflow.com/questions/58997230/cmake-project-fails-to-find-shared-library
target_link_libraries(${PROJECT_NAME}_node "-Wl,--disable-new-dtags") 

# Install binaries
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
