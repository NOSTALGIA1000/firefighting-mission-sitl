cmake_minimum_required(VERSION 3.0.2)
project(firefighting_mission)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(FILES
  DropResult.msg
  MissionEvent.msg
  TargetDetection.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  rospy
  sensor_msgs
  std_msgs
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_package_metadata.py)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_world_generator.py)
endif()

catkin_install_python(PROGRAMS scripts/generate_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
