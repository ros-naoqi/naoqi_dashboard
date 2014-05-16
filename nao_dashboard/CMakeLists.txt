cmake_minimum_required(VERSION 2.8)
project(nao_dashboard)

find_package(catkin)

find_package(catkin REQUIRED diagnostic_aggregator diagnostic_msgs pluginlib)
catkin_package(DEPENDS 
               LIBRARIES nao_diagnostic_analyzers
)

catkin_python_setup()

install(DIRECTORY config icons launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/nao_dashboard DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
