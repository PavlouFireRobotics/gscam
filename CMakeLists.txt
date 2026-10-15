cmake_minimum_required(VERSION 3.5)

project(gscam)

# System Dependencies
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0 gstreamer-video-1.0)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

add_library(gscam SHARED src/gscam.cpp)
target_compile_features(gscam PUBLIC cxx_std_14)
target_include_directories(gscam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS})
target_link_libraries(gscam
  ${ament_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
  ${GST_APP_LIBRARIES})
# Foxy dependencies do not all export the newer namespaced CMake targets.
# Keep modern target linking for distributions where ament_target_dependencies
# is no longer available.
if(TARGET class_loader::class_loader AND TARGET image_transport::image_transport
    AND TARGET rclcpp::rclcpp AND TARGET rclcpp_components::component
    AND TARGET camera_calibration_parsers::camera_calibration_parsers
    AND TARGET camera_info_manager::camera_info_manager AND sensor_msgs_TARGETS)
  target_link_libraries(gscam
    class_loader::class_loader
    image_transport::image_transport
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    camera_calibration_parsers::camera_calibration_parsers
    camera_info_manager::camera_info_manager)
else()
  ament_target_dependencies(gscam
    class_loader image_transport rclcpp rclcpp_components sensor_msgs
    camera_calibration_parsers camera_info_manager)
endif()

if(image_transport_VERSION VERSION_LESS 6.3.0)
  target_compile_definitions(gscam
    PUBLIC
    USE_OLD_RMW_QOS_IMAGE_TRANSPORT=1
  )
endif()

rclcpp_components_register_node(gscam
  PLUGIN "gscam::GSCam"
  EXECUTABLE gscam_node
)

set(node_plugins "${node_plugins}gscam::GSCam;$<TARGET_FILE:gscam>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(appsink_test test/appsink_test.cpp)
  if(TARGET appsink_test)
    target_link_libraries(appsink_test gscam)
  endif()
  string(CONCAT smoke_pipeline
    "videotestsrc is-live=true pattern=red ! "
    "capsfilter caps=video/x-raw,width=641,height=480,framerate=30/1 ! videoconvert")
  ament_add_gtest(
    smoke_test
    test/smoke_test.cpp
    TIMEOUT 15
    ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs ROS_LOCALHOST_ONLY=1
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp "GSCAM_CONFIG=${smoke_pipeline}"
  )
  if(TARGET smoke_test)
    target_link_libraries(smoke_test gscam)
  endif()
  ament_add_gtest(stream_test test/stream_test.cpp TIMEOUT 30
    ENV ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs ROS_LOCALHOST_ONLY=1
        RMW_IMPLEMENTATION=rmw_fastrtps_cpp)
  if(TARGET stream_test)
    target_link_libraries(stream_test gscam)
  endif()
endif()

ament_export_dependencies(class_loader image_transport rclcpp rclcpp_components
  sensor_msgs camera_calibration_parsers camera_info_manager)

ament_export_include_directories(include)

ament_export_targets(export_gscam)

ament_export_libraries(gscam)

# Install directives

install(TARGETS gscam
  EXPORT export_gscam
  DESTINATION lib
)

install(
  DIRECTORY "examples"
  DESTINATION "share/${PROJECT_NAME}"
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
