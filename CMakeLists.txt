cmake_minimum_required(VERSION 2.8.3)
project(projected_interface_builder)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure sensor_msgs rospy geometry_msgs projector_interface)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/InterfaceAdjustments.cfg
)

catkin_package(
    CATKIN_DEPENDS dynamic_reconfigure sensor_msgs rospy geometry_msgs projector_interface std_srvs
)

install(DIRECTORY cfg icons DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS interface_builder.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories( ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})