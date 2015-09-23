#ifndef ORGANIZED_FAST_MESH_H_
#define ORGANIZED_FAST_MESH_H_

#include "organized_fast_mesh_generator.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <mesh_msgs/TriangleMeshStamped.h>
#include <lvr_ros/lvr_ros_conversions.h>

#include <geometry/HalfEdgeMesh.hpp>
#include <path_planning/GraphHalfEdgeMesh.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

typedef lvr::ColorVertex<float, int> VertexType;
typedef lvr::Normal<float> NormalType;
typedef unsigned char uchar;

class OrganizedFastMesh{

  public:
    OrganizedFastMesh(ros::NodeHandle &nh);
  private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher mesh_pub_;
    ros::Publisher marker_pub_;
    //ros::Publisher p_normals_pub_;
    void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud);
};

#endif /* organized_fast_mesh.h */
