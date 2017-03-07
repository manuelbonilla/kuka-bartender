cmake_minimum_required(VERSION 2.8.3)
project(bartender_control)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	message_generation 
	lwr_controllers
  controller_interface
)

add_definitions (-fpermissive -std=c++11)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  bartender_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
	roscpp 
  controller_interface
	std_msgs 
	message_generation  
	lwr_controllers
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/one_task_inverse_kinematics.cpp /home/andrea/catkin_ws/src/kuka-lwr/lwr_controllers/src/KinematicChainControllerBase.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES bartender_control_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})