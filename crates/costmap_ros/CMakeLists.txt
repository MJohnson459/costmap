cmake_minimum_required(VERSION 3.16)
project(costmap_ros NONE)

# ON installs what ROS needs to find the package without ament_cmake, so the
# build needs CMake and a Rust toolchain and no ROS install. `cpack` then
# writes the prefix as costmap_ros-VERSION-ARCH.tar.gz.
option(COSTMAP_ROS_STANDALONE "Build without ament_cmake" OFF)
if(NOT COSTMAP_ROS_STANDALONE)
  find_package(ament_cmake REQUIRED)
endif()

# The nodes are Rust binaries. cargo builds them from the workspace this
# package sits in, with the versions in the workspace's Cargo.lock.
find_program(CARGO cargo REQUIRED)
get_filename_component(WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(CARGO_TARGET_DIR "${CMAKE_CURRENT_BINARY_DIR}/cargo")
# With a target named, as conda's Rust toolchain does, cargo adds a directory.
if(DEFINED ENV{CARGO_BUILD_TARGET})
  set(BUILT "${CARGO_TARGET_DIR}/$ENV{CARGO_BUILD_TARGET}/release")
else()
  set(BUILT "${CARGO_TARGET_DIR}/release")
endif()
# The two nodes, and the tool that writes a ROS map as a cost GeoTIFF.
set(PROGRAMS costmap_window_server costmap_planner_server costmap-from-ros-map)
set(BIN_FLAGS "")
foreach(program ${PROGRAMS})
  list(APPEND BIN_FLAGS --bin ${program})
endforeach()

# cargo decides what is out of date, so the target runs on every build.
# The workspace's release profile keeps debug information, which makes each
# binary 20 times larger; installed binaries are built without it.
add_custom_target(cargo_build ALL
  COMMAND ${CMAKE_COMMAND} -E env
    CARGO_PROFILE_RELEASE_DEBUG=false
    CARGO_PROFILE_RELEASE_STRIP=true
    ${CARGO} build --release --locked
      --package costmap_ros --package costmap-geo ${BIN_FLAGS}
      --manifest-path "${WORKSPACE}/Cargo.toml"
      --target-dir "${CARGO_TARGET_DIR}"
  USES_TERMINAL
  VERBATIM)

foreach(program ${PROGRAMS})
  install(PROGRAMS "${BUILT}/${program}"
    DESTINATION lib/${PROJECT_NAME})
endforeach()
install(DIRECTORY behavior_trees launch DESTINATION share/${PROJECT_NAME})

if(COSTMAP_ROS_STANDALONE)
  # The parts of ament_package() that `ros2 run` and `ros2 launch` rely on:
  # the manifest, the package's entry in the ament index, and a setup file at
  # the path ament gives it.
  install(FILES package.xml standalone/local_setup.bash
    DESTINATION share/${PROJECT_NAME})
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ament_index/${PROJECT_NAME}" "")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ament_index/${PROJECT_NAME}"
    DESTINATION share/ament_index/resource_index/packages)

  file(READ package.xml PACKAGE_XML)
  string(REGEX MATCH "<version>([^<]+)</version>" _ "${PACKAGE_XML}")
  set(CPACK_GENERATOR TGZ)
  set(CPACK_PACKAGE_FILE_NAME
    "${PROJECT_NAME}-${CMAKE_MATCH_1}-${CMAKE_SYSTEM_PROCESSOR}")
  # The archive holds one directory, costmap_ros.
  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/${PROJECT_NAME}")
  include(CPack)
else()
  ament_package()
endif()
