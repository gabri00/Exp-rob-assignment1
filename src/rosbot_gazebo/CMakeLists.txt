cmake_minimum_required(VERSION 2.8.3)
project(rosbot_gazebo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	cv_bridge
	image_transport
	sensor_msgs
)

find_package (OpenCV REQUIRED)

catkin_package(
	DEPENDS OpenCV
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

# install(DIRECTORY launch
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

# install(DIRECTORY worlds
# 	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#  add_executable(${PROJECT_NAME}_node src/example.cpp)
#  add_executable(${PROJECT_NAME}_node_t src/example_transport.cpp)
#  add_executable(random_robot src/random_robot.cpp)

## Specify libraries to link a library or executable target against
#   target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#  )

#  target_link_libraries(${PROJECT_NAME}_node_t
#    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#  )

# target_link_libraries(random_robot
#    ${catkin_LIBRARIES} 
#  )

# install(PROGRAMS
#    scripts/example_ball.py scripts/example.py scripts/example_t.py scripts/robot_following.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )