cmake_minimum_required(VERSION 2.4.6)

project(organized_fast_mesh)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
  sensor_msgs
  shape_msgs
  mesh_msgs
  lvr_ros
)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)


add_definitions("-frounding-math")
add_definitions("-std=c++0x")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-D_USE_PCL_)


FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

find_package(PCL 1.0 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

### LVR
set(LVR_DIR "/home/robot/ros/muffin-dry/mapping/lvr")
set(LVR_INCLUDE_DIRS
  ${LVR_DIR}/ext/psimpl
  ${LVR_DIR}/ext/rply
  ${LVR_DIR}/ext/stann
  ${LVR_DIR}/ext
  ${LVR_DIR}/include/liblvr
)
set(LVR_LIBRARY_DIRS
  ${LVR_DIR}/lib
)
set(_LVR_LIBRARIES
  lvr
)
foreach(lib ${_LVR_LIBRARIES})
  find_library(${lib}_FOUND ${lib} HINTS ${LVR_LIBRARY_DIRS})
  if(${lib}_FOUND)
    list(APPEND LVR_LIBRARIES ${${lib}_FOUND})
  else()
    message( FATAL_ERROR "${lib} not found" )
  endif()
endforeach()

### end lvr

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lvr_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${LVR_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  ${LVR_LIBRARY_DIRS}
)

add_service_files(
  FILES
  OrganizedFastMeshSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  shape_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros lvr_ros
  INCLUDE_DIRS include 
  DEPENDS PCL OPENGL GLUT CGAL LVR
) 

add_executable(organized_fast_mesh src/organized_fast_mesh.cpp src/organized_fast_mesh_generator.cpp)

target_link_libraries(organized_fast_mesh
  ${catkin_LIBRARIES}
  ${OPENGL_glu_LIBRARY}
  ${LVR_LIBRARIES}
)

add_dependencies(organized_fast_mesh ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

install(
  DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

