cmake_minimum_required(VERSION 2.8.3)
project(gcs_agent)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	sensor_msgs
	geometry_msgs
        sensor_fusion_comm
        mavlink
        libmavconn
        mavros_msgs
	svo_msgs
	glog_catkin
	cmake_modules
)

find_package(Eigen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES mavros
  CATKIN_DEPENDS roscpp geometry_msgs svo_msgs mavros_msgs sensor_fusion_comm
  DEPENDS Boost Eigen
)

add_definitions(-DMAVLINK_DIALECT=ardupilotmega)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Eigen_INCLUDE_DIRS}
)

add_executable(gcs_agent main.cpp gcsagent.cpp launcher.cpp svo.cpp msf.cpp) #mavros.cpp)
#add_dependencies(gcs_agent mavros_msgs)
add_dependencies(gcs_agent mavros_msgs_generate_messages_cpp)
add_dependencies(gcs_agent geometry_msgs_generate_messages_cpp)

target_link_libraries(gcs_agent
  ${catkin_LIBRARIES}
)
