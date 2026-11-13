cmake_minimum_required(VERSION 3.16)
project(tiled_map_server NONE)

# ON installs what ROS needs to find the package without ament_cmake, so the
# build needs CMake and a Rust toolchain and no ROS install. `cpack` then
# writes the prefix as a Debian package.
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
# The node, and the tool that writes a ROS map as a cost GeoTIFF.
set(PROGRAMS tiled_map_server costmap-from-ros-map)
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
      --package costmap_ros ${BIN_FLAGS}
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
  # the manifest and the package's entry in the ament index.
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/ament_index/${PROJECT_NAME}" "")
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/ament_index/${PROJECT_NAME}"
    DESTINATION share/ament_index/resource_index/packages)

  # `cpack` writes ros-DISTRO-tiled-map-server_VERSION_ARCH.deb, which installs
  # into /opt/ros/DISTRO beside the distribution's own packages.
  set(ROS_DISTRO "lyrical" CACHE STRING "ROS distribution the .deb installs into")
  file(READ package.xml PACKAGE_XML)
  string(REGEX MATCH "<version>([^<]+)</version>" _ "${PACKAGE_XML}")
  set(CPACK_GENERATOR DEB)
  set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/ros/${ROS_DISTRO}")
  set(CPACK_PACKAGE_NAME "ros-${ROS_DISTRO}-tiled-map-server")
  set(CPACK_PACKAGE_VERSION "${CMAKE_MATCH_1}")
  set(CPACK_PACKAGE_CONTACT "Michael Johnson <mjohnson459@gmail.com>")
  set(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "Serves Nav2 a window of a large map and routes across it, over Zenoh")
  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)
  set(CPACK_DEBIAN_PACKAGE_SECTION misc)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS
    "libc6 (>= 2.35), libgcc-s1, ros-${ROS_DISTRO}-launch-ros, ros-${ROS_DISTRO}-rmw-zenoh-cpp")
  include(CPack)
else()
  ament_package()
endif()
