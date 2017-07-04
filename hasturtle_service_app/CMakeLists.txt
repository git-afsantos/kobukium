cmake_minimum_required(VERSION 2.8.3)
project(hasturtle_service_app)

find_package(catkin REQUIRED COMPONENTS
  rospy geometry_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS rospy message_runtime geometry_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS bin/hasturtle_service
                               scripts/test_fake_robot
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY param/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

install(DIRECTORY html/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/html
)
