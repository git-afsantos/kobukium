cmake_minimum_required(VERSION 2.8.3)
project(kobuki_noros)

find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs kobuki_msgs nav_msgs tf
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy message_runtime geometry_msgs kobuki_msgs nav_msgs tf
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS bin/noros_server
                               bin/noros_programmable
                               scripts/noros_turtlesim_test
                               scripts/noros_kobuki_test
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY param/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)
