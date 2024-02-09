cmake_minimum_required(VERSION 3.8)
project(path_following_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  trajectory_msgs
  angles
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add path_following_controller library related compile commands
generate_parameter_library(path_following_controller_parameters
  src/path_following_controller.yaml
  include/path_following_controller/validate_path_following_controller_parameters.hpp
)
add_library(
  path_following_controller
  SHARED
  src/path_following_controller.cpp
  src/trajectory.cpp
)
target_include_directories(path_following_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(path_following_controller path_following_controller_parameters)
ament_target_dependencies(path_following_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(path_following_controller PRIVATE "PATH_FOLLOWING_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface path_following_controller.xml)

install(
  TARGETS
  path_following_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  set(THIS_PACKAGE_BUILD_DEPENDS
    ament_cmake_gmock
    controller_manager
    ros2_control_test_assets
  )
  foreach(Dependency IN ITEMS ${THIS_PACKAGE_BUILD_DEPENDS})
  find_package(${Dependency} REQUIRED)
  endforeach()
  
  ### test loading controller
  ament_add_gmock(test_load_path_following_controller
    test/test_load_path_following_controller.cpp
  )
  ament_target_dependencies(test_load_path_following_controller 
    ${THIS_PACKAGE_BUILD_DEPENDS}
  )

  ### test trajectory management
  ament_add_gmock(test_trajectory
    test/test_trajectory.cpp
  )
  target_link_libraries(test_trajectory 
    path_following_controller
  )

  
  ### controller lifecycle test
  ament_add_gmock(test_path_following_controller
    test/test_path_following_controller
  )
  ament_target_dependencies(test_path_following_controller
    ${THIS_PACKAGE_BUILD_DEPENDS}
  )
  target_link_libraries(test_path_following_controller
    path_following_controller
  )
  


endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  path_following_controller
)

ament_package()
