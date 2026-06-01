cmake_minimum_required(VERSION 3.5)
project(auction_behavior VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PROJECT_DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  rclcpp_cascade_lifecycle
  lifecycle_msgs
  pluginlib
  cs4home_core
  ca_structure
  as2_msgs
  geometry_msgs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

include_directories(
  include
  include/${PROJECT_NAME}
)

# ── Plugin base library ──────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_plugin_base SHARED
  include/${PROJECT_NAME}/auction_behavior_plugin_base.hpp
)
set_target_properties(${PROJECT_NAME}_plugin_base PROPERTIES LINKER_LANGUAGE CXX)
ament_target_dependencies(${PROJECT_NAME}_plugin_base ${PROJECT_DEPENDENCIES})

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin_base)
ament_export_targets(${PROJECT_NAME}_plugin_base)

install(
  TARGETS ${PROJECT_NAME}_plugin_base
  EXPORT  ${PROJECT_NAME}_plugin_base
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ── Plugins ──────────────────────────────────────────────────────────────────
set(PLUGIN_LIST
  greedy_sequential
  cbba
  coordinate_item
)

foreach(PLUGIN ${PLUGIN_LIST})
  add_subdirectory(plugins/${PLUGIN})
endforeach()

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# ── Behavior node (Afferent + Core + CognitiveModule) ────────────────────────
set(BEHAVIOR_SOURCES
  src/auction_behavior_afferent.cpp
  src/auction_behavior_core.cpp
  src/auction_behavior.cpp
  src/auction_behavior_node.cpp
)

add_executable(${PROJECT_NAME}_node ${BEHAVIOR_SOURCES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_plugin_base)
ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})
target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${PROJECT_DEPENDENCIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  file(GLOB_RECURSE EXCLUDE_FILES build/* install/*)
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(tests)
endif()

ament_package()
