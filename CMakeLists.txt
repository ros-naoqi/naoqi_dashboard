cmake_minimum_required(VERSION 2.8)
project(naoqi_dashboard)

find_package(catkin)

catkin_package()

catkin_python_setup()

install(DIRECTORY config icons launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/naoqi_dashboard DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
