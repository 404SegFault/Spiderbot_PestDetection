cmake_minimum_required(VERSION 3.0.2)
project(spiderbot_teleop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/web_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch www
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
