#include <organized_fast_mesh.h>
#include <stdlib.h>
#include <time.h>

OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
  : nh_(nh)
{
  cloud_sub_ = nh_.subscribe("input_cloud", 20, &OrganizedFastMesh::pointCloud2Callback, this);
  mesh_pub_ = nh_.advertise<mesh_msgs::TriangleMeshStamped>("segment_mesh", 1);
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("path", 10);
}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr
    &cloud){

  ROS_INFO("got organized point cloud");

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(*cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_organized;
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_organized);
  

  OrganizedFastMeshGenerator ofmg(cloud_organized);
  lvr::GraphHalfEdgeMesh<VertexType, NormalType> hem;
  ofmg.getMesh(hem);

  std::list<int> path;

  int start = hem.getVertices().size()-1; // hem.getRandomVertexID();
  int goal = hem.getRandomVertexID();

  ROS_INFO("start astar from the face with the id %d to the id %d", start, goal);
  bool found_path = hem.vertexGraphDijkstra(start, goal, path);
  if(found_path){
    ROS_INFO("Path found");
    
    // path to marker: Linestrip
    visualization_msgs::Marker line_strip;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header.frame_id = cloud->header.frame_id;
    line_strip.header.stamp =cloud->header.stamp;
    line_strip.ns = "astar shortest path";
    line_strip.action  = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 0;
    line_strip.scale.x = 0.1;
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;

    for(std::list<int>::iterator iter=path.begin(); iter != path.end(); ++iter){
      //lvr::HalfEdgeMesh<VertexType, NormalType>::FaceVector& faces = hem.getFaces();
      lvr::HalfEdgeMesh<VertexType, NormalType>::VertexVector& vertices = hem.getVertices();
      VertexType vertex = vertices[*iter]->m_position;//faces[*iter]->getCentroid();
      geometry_msgs::Point p;
      p.x = vertex.x;
      p.y = vertex.y;
      p.z = vertex.z;
      line_strip.points.push_back(p);
    }

    ROS_INFO("Publish Path as Marker: Linestrip");
    marker_pub_.publish(line_strip);
  }else{
    ROS_INFO("No Path found");
  }


  mesh_msgs::TriangleMeshStamped mesh_msg;
  mesh_msg.header.frame_id = cloud->header.frame_id;
  mesh_msg.header.stamp = cloud->header.stamp;
    
  bool success = lvr_ros::fromMeshBufferToTriangleMesh(hem.meshBuffer(), mesh_msg.mesh);
  
  size_t vertices_cnt = mesh_msg.mesh.vertices.size();
  size_t triangles_cnt = mesh_msg.mesh.triangles.size();
  mesh_msg.mesh.vertex_colors.resize(vertices_cnt);
  mesh_msg.mesh.triangle_colors.resize(triangles_cnt);
  for(int i=0; i< vertices_cnt; i++){
    mesh_msg.mesh.vertex_colors[i].r = ((float)i)/vertices_cnt;
    mesh_msg.mesh.vertex_colors[i].g = 1.0-((float)i)/vertices_cnt;
    mesh_msg.mesh.vertex_colors[i].b = 0.0f;
    mesh_msg.mesh.vertex_colors[i].a = 1.0f;
  }
  
  srand (static_cast <unsigned> (time(0)));

  float r, g, b;

  for(int i=0; i< triangles_cnt; i++){
    r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    g = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    b = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    mesh_msg.mesh.triangle_colors[i].r = r;
    mesh_msg.mesh.triangle_colors[i].g = g;
    mesh_msg.mesh.triangle_colors[i].b = b;
    mesh_msg.mesh.triangle_colors[i].a = 1.0;
  }


  ROS_INFO("Publish organized fast mesh...");
  mesh_pub_.publish(mesh_msg);
}

int main(int args, char** argv){
  ros::init(args, argv, "organized_fast_mesh");
  ros::NodeHandle nh;
  OrganizedFastMesh ofm(nh);
  ros::spin();
}
