cmake_minimum_required(VERSION 3.0.2)
project(firefighting_mission)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

add_message_files(FILES
  DropResult.msg
  MissionEvent.msg
  TargetDetection.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  LIBRARIES fire_payload_plugin
  CATKIN_DEPENDS
  geometry_msgs
  gazebo_msgs
  mavros_msgs
  cv_bridge
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  DEPENDS GAZEBO
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(fire_payload_plugin src/payload_plugin.cpp)
add_dependencies(fire_payload_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(fire_payload_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_package_metadata.py)
  catkin_add_nosetests(test/test_navigation.py)
  catkin_add_nosetests(test/test_orchestration.py)
  catkin_add_nosetests(test/test_perception.py)
  catkin_add_nosetests(test/test_payload.py)
  catkin_add_nosetests(test/test_safety.py)
  catkin_add_nosetests(test/test_scoring.py)
  catkin_add_nosetests(test/test_state_machine.py)
  catkin_add_nosetests(test/test_world_generator.py)
  add_rostest(test/navigation.test)
  add_rostest(test/payload_drop.test)
endif()

catkin_install_python(PROGRAMS
  scripts/generate_world.py
  scripts/navigator_node.py
  scripts/safety_monitor_node.py
  scripts/target_detector_node.py
  scripts/payload_controller_node.py
  scripts/mission_manager_node.py
  scripts/mission_recorder_node.py
  scripts/mavros_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/run_mission.sh scripts/start_sitl.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
