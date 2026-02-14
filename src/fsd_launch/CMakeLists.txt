cmake_minimum_required(VERSION 3.0.2)
project(fsd_launch)

find_package(catkin REQUIRED)
catkin_package()

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

# Install scripts
catkin_install_python(PROGRAMS
  scripts/param_snapshot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
