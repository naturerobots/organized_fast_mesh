#include <organized_fast_mesh.h>

OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
  : nh_(nh)
{
  cloud_sub_ = nh_.subscribe("input_cloud", 20, &OrganizedFastMesh::pointCloud2Callback, this);
}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr
    &cloud){

  ROS_INFO("Cloud-Height: %d", cloud->height);
  ROS_INFO("Cloud-Width: %d", cloud->width);

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_organized (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromPCLPointCloud2(pcl_cloud, *cloud_organized);

  

}

int main(int args, char** argv){
  ros::init(args, argv, "organized_fast_mesh");
  ros::NodeHandle nh;
  OrganizedFastMesh ofm(nh);
  ros::spin();
}
