#include <organized_fast_mesh.h>

OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
  : nh_(nh)
{
  cloud_sub_ = nh_.subscribe("input_cloud", 20, &OrganizedFastMesh::pointCloud2Callback, this);
  mesh_pub_ = nh_.advertise<lvr_ros::Mesh>("organized_mesh/mesh", 1);
  p_normals_pub_ = nh_.advertise<lvr_ros::PointNormals>("organized_mesh/point_normals", 1);

}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr
    &cloud){

  ROS_INFO("Cloud-Height: %d", cloud->height);
  ROS_INFO("Cloud-Width: %d", cloud->width);

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_organized;
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_organized);

  
  OrganizedFastMeshGenerator ofmg(cloud_organized);
  lvr::HalfEdgeMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> > hem;
  ofmg.getMesh(hem);


  lvr_ros::Mesh mesh;
  lvr_ros::PointNormals normals;
  mesh.header.frame_id = cloud->header.frame_id;
  mesh.header.stamp = cloud->header.stamp;
  normals.header.frame_id = cloud->header.frame_id;
  normals.header.stamp = cloud->header.stamp;
    
  lvr_ros_converter::LvrRosConverter converter;
  mesh.mesh = converter.convertMeshBufferToMeshMessage(hem.meshBuffer(), cloud->header.stamp, cloud->header.frame_id);

  ROS_INFO("Publish organized fast mesh...");
  mesh_pub_.publish(mesh);
}

int main(int args, char** argv){
  ros::init(args, argv, "organized_fast_mesh");
  ros::NodeHandle nh;
  OrganizedFastMesh ofm(nh);
  ros::spin();
}
