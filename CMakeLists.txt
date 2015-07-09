cmake_minimum_required(VERSION 2.4.6)

project(organized_fast_mesh)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
  sensor_msgs
  shape_msgs
  lvr_ros
)

find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(CGAL REQUIRED)
find_package(lvr_ros REQUIRED)


add_definitions("-frounding-math")
add_definitions("-std=c++0x")
add_definitions(${PCL_DEFINITIONS})
add_definitions(-D_USE_PCL_)


FIND_PATH(OPENGL_INC gl.h /usr/include/GL)
include_directories(${OPENGL_INC})

find_package(PCL 1.0 REQUIRED)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${lvr_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  /include/lvr
  /home/robot/ros/muffin-dry/mapping/lvr/ext/psimpl
  /home/robot/ros/muffin-dry/mapping/lvr/ext/rply
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${CGAL_LIBRARY_DIRS}
  ${OPENGL_LIBRARY_DIRS}
  ${GLUT_LIBRARY_DIRS}
  /home/robot/ros/muffin-dry/mapping/lvr/lib/
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
  DEPENDS PCL OPENGL GLUT CGAL
) 

add_executable(organized_fast_mesh src/organized_fast_mesh.cpp src/organized_fast_mesh_generator.cpp)

target_link_libraries(organized_fast_mesh
  ${catkin_LIBRARIES}
  ${OPENGL_glu_LIBRARY}
  lvr
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

