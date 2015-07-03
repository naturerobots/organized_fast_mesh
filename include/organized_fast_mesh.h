#ifndef ORGANIZED_FAST_MESH_H_
#define ORGANIZED_FAST_MESH_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class OrganizedFastMesh{

  public:
    OrganizedFastMesh(ros::NodeHandle &nh);
  private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    
    void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud);
};

#endif /* organized_fast_mesh.h */
