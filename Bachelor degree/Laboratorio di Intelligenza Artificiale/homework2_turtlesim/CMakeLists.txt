cmake_minimum_required(VERSION 3.0.2)
project(turtlesim)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES 
                  Color.msg Pose.msg Circle.msg)
add_service_files(DIRECTORY srv FILES
                  Kill.srv
                  SetPen.srv
                  Spawn.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv
  		  SpawnCircle.srv
		  RemoveCircle.srv
		  GetCircles.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
  include/turtlesim/turtle_frame.h
)

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})


add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)

add_executable(spawn_circle_server tutorials/spawn_circle_server.cpp)
target_link_libraries(spawn_circle_server ${catkin_LIBRARIES})
add_dependencies(spawn_circle_server turtlesim_gencpp)

add_executable(spawn_circle_client tutorials/spawn_circle_client.cpp)
target_link_libraries(spawn_circle_client ${catkin_LIBRARIES})
add_dependencies(spawn_circle_client turtlesim_gencpp)

add_executable(get_circles_client tutorials/get_circles_client.cpp)
target_link_libraries(get_circles_client ${catkin_LIBRARIES})
add_dependencies(get_circles_client turtlesim_gencpp)

add_executable(remove_circle_client tutorials/remove_circle_client.cpp)
target_link_libraries(remove_circle_client ${catkin_LIBRARIES})
add_dependencies(remove_circle_client turtlesim_gencpp)

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
