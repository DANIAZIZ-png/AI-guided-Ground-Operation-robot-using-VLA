cmake_minimum_required(VERSION 3.8)
project(vla_bringup)

find_package(ament_cmake REQUIRED)

# ament_cmake rather than ament_python: this package ships no Python modules,
# only files to install. CMake installs files from outside its own directory
# with plain paths, which is what lets launch/, config/ and maps/ stay at the
# repository root instead of being moved in here -- so $VLA_LAUNCH_DIR,
# $VLA_CONFIG_DIR and the operator console's launch buttons keep working.
# setuptools data_files does not handle ".." reliably.

# REALPATH, not a plain "..": this package is normally reached through a symlink
# from a colcon workspace (ros2_ws/src/vla_bringup -> <repo>/vla_bringup). A
# lexical ".." from that symlink lands in the workspace's src/ directory, not in
# the repository, and the install would then either fail confusingly or silently
# pick up nothing.
#
# It has to be done in TWO steps. REALPATH on ".../vla_bringup/.." normalises
# the ".." lexically BEFORE resolving the symlink, which lands back in the
# workspace's src/. Resolving the package directory first, then going up from
# the real location, is what actually works.
get_filename_component(_pkg_dir "${CMAKE_CURRENT_LIST_DIR}" REALPATH)
get_filename_component(REPO "${_pkg_dir}/.." REALPATH)
message(STATUS "vla_bringup: repository root resolved to ${REPO}")

# Fail at CONFIGURE time with a readable message if the repository layout is not
# what is expected. Without this the first sign of trouble is a cmake install
# error naming a path nobody recognises.
foreach(_required
    "launch/nav2_hw_composed.launch.py"
    "launch/slam_hw.launch.py"
    "config/slam_vla.yaml"
    "config/fastdds_hw_wifi_only.xml"
    "maps/warehouse_map.yaml")
  if(NOT EXISTS "${REPO}/${_required}")
    message(FATAL_ERROR
      "vla_bringup expected to find ${_required} under ${REPO}. "
      "This package installs the repository's own launch files, parameters and "
      "map; it must stay inside the repository (symlink it into a colcon "
      "workspace rather than copying it out).")
  endif()
endforeach()

# This package's own top-level orchestrators.
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# The repository's hardware launch files, installed alongside them. These are
# the ones that remap /parameter_events and /rosout off the Wi-Fi link; the
# stock turtlebot4_navigation files do not, and launching those floods the
# robot at ~7,400 packets/s.
install(FILES
  "${REPO}/launch/nav2_hw.launch.py"
  "${REPO}/launch/nav2_hw_composed.launch.py"
  "${REPO}/launch/slam_hw.launch.py"
  DESTINATION share/${PROJECT_NAME}/launch)

# Parameters, DDS profiles and the RViz layout.
# fastdds_hw_wifi_only.xml is the profile in use (super client + interface
# whitelist); fastdds_super_client.xml is its predecessor, kept for reference.
install(FILES
  "${REPO}/config/slam_vla.yaml"
  "${REPO}/config/nav2_hw_slow.yaml"
  "${REPO}/config/vla_hardware.rviz"
  "${REPO}/config/fastdds_hw_wifi_only.xml"
  "${REPO}/config/fastdds_super_client.xml"
  DESTINATION share/${PROJECT_NAME}/config)

# The saved SLAM map.
install(FILES
  "${REPO}/maps/warehouse_map.yaml"
  "${REPO}/maps/warehouse_map.pgm"
  DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
