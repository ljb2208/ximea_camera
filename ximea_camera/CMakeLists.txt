cmake_minimum_required(VERSION 2.8.3)
project(ximea_camera)

set(XIMEA_INCLUDE_PATH /usr/include)
set(XIMEA_LIBRARY_PATH /usr/lib)
set(XIMEA_LIBRARY m3api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  sensor_msgs
  camera_info_manager
  roslint
  message_generation
  nodelet
  image_exposure_msgs
  wfov_camera_msgs
)

find_package(Boost REQUIRED COMPONENTS system thread)


generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  LIBRARIES ximea_camera
  CATKIN_DEPENDS roscpp nodelet message_runtime sensor_msgs image_exposure_msgs
  DEPENDS system_lib TinyXML
  DEPENDS boost
)

include_directories(include)

include_directories(
  ${XIMEA_INCLUDE_PATH}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(XIMEA_SRC
    src/ximea_driver.cpp
    src/ximea_ros_driver.cpp
    src/ximea_ros_cluster.cpp
    src/ximea.cpp    
)

add_library(XimeaCameraNodelet src/ximea_nodelet.cpp ${XIMEA_SRC})
target_link_libraries(XimeaCameraNodelet ${XIMEA_LIBRARY} ${Boost_LIBRARIES} yaml-cpp ${catkin_LIBRARIES})
add_dependencies(XimeaCameraNodelet ${catkin_EXPORTED_TARGETS})

add_executable(ximea_camera_node ${XIMEA_SRC})
target_link_libraries(
    ximea_camera_node
    ${XIMEA_LIBRARY}
    ${catkin_LIBRARIES}
    yaml-cpp
    ${Boost_LIBRARIES}
)

# ROSLINT
set(ROSLINT_CPP_OPTS  "--filter=-whitespace/line_length,-readability/todo" )
roslint_cpp()


install(FILES nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)