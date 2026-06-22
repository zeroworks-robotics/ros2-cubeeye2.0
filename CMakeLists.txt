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

# 벤더 SDK 공유 라이브러리 설치 (libCubeEye + 동봉 thirdparty: libusb/ffmpeg/live555/opencv3.4).
# link_directories 로 빌드 시 링크만 하고 install 을 안 하면 merge-install(.deb)에 .so 가
# 빠져 런타임 로드 실패한다(= symlink-install 로만 우연히 동작). lib/ 에 설치하면
# ament library_path 훅이 <prefix>/lib 를 LD_LIBRARY_PATH 에 추가 → 정상 로드 + self-contained deb.
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/lib/
    ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libusb/lib/
    ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libffmpeg/lib/
    ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/liblive555/lib/Release/
    ${CMAKE_CURRENT_SOURCE_DIR}/cubeeye2.0/thirdparty/libopencv/lib/
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
