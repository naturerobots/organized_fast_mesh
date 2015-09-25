#include <organized_fast_mesh.h>
#include <stdlib.h>
#include <time.h>

OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
  : nh_(nh)
{
  cloud_sub_ = nh_.subscribe("input_cloud", 20, &OrganizedFastMesh::pointCloud2Callback, this);
  mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("organized_mesh", 1);
}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  if(cloud->height < 2){
    ROS_WARN("Received unorganized point cloud!");
    return;
  }
  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_organized;
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_organized);
  
  OrganizedFastMeshGenerator ofmg(cloud_organized);
  lvr::HalfEdgeMesh<VertexType, NormalType> hem;
  ofmg.getMesh(hem);
  
  mesh_msgs::TriangleMeshStamped mesh_msg;
  mesh_msg.header.frame_id = cloud->header.frame_id;
  mesh_msg.header.stamp = cloud->header.stamp;
  if(lvr_ros::fromMeshBufferToTriangleMesh(hem.meshBuffer(), mesh_msg.mesh)){
    mesh_pub_.publish(mesh_msg);
  }
}

int main(int args, char** argv){
  ros::init(args, argv, "organized_fast_mesh");
  ros::NodeHandle nh;
  ros::spin();
}
