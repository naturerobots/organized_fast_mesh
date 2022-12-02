/*
 *  Software License Agreement (BSD License)
 *
 *  Robot Operating System code by the University of Osnabrück
 *  Copyright (c) 2015, University of Osnabrück
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above 
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *  TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *
 *  organized_fast_mesh.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */
#include "organized_fast_mesh.h"
#include "organized_fast_mesh_generator.h"
#include <lvr_ros/conversions.h>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/geometry/ColorVertex.hpp>


#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>



typedef lvr2::ColorVertex<float, int> VertexType;
typedef lvr2::Normal<float> NormalType;


OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
  : nh_(nh)
{

    cloud_sub_ = nh_.subscribe("/os_cloud_node/points", 20, &OrganizedFastMesh::pointCloud2Callback, this);


  mesh_pub_ = nh_.advertise<mesh_msgs::MeshGeometryStamped>("organized_mesh", 1);
  service_ = nh_.advertiseService("organized_fast_mesh", &OrganizedFastMesh::generateOrganizedFastMeshSrv, this);

  ros::NodeHandle p_nh_("~");
  p_nh_.param("edge_threshold", edge_threshold,0.5);
  p_nh_.param("fillup_base_hole", fillup_base_hole, false);

}
bool OrganizedFastMesh::generateOrganizedFastMesh(
  const sensor_msgs::PointCloud2& cloud, mesh_msgs::MeshGeometryStamped& mesh_msg)
{






    if(cloud.height < 2){
    ROS_WARN("Received unorganized point cloud!");
    return false;
  }

  pcl::PCLPointCloud2 pcl_cloud;
  pcl_conversions::toPCL(cloud, pcl_cloud);
  pcl::PointCloud<pcl::PointNormal> cloud_organized;
  pcl::fromPCLPointCloud2(pcl_cloud, cloud_organized);

/*
  lvr2::PointCloud lvr2_cloud;
  lvr2::PointBuffer PointBuffer;
  lvr_ros::fromPointCloud2ToPointBuffer(cloud )
*/

  OrganizedFastMeshGenerator ofmg(cloud_organized);
  ROS_INFO("size of cloud_organized: %d", cloud_organized.size());
  ROS_INFO("size of pcloud: %d",cloud.height*cloud.width );

  ofmg.setEdgeThreshold(edge_threshold);
  //old version
  //lvr2::HalfEdgeMesh<VertexType, NormalType> hem;

  lvr2::MeshBuffer hem;

  ofmg.getMesh(hem);

  std::vector<int> contour, fillup_indices;
  if(fillup_base_hole){
    ofmg.getContour(contour);
    ROS_INFO("base hole contour Size: %d", contour.size());
    ofmg.fillContour(contour, hem, fillup_indices);
  }

 bool success = lvr_ros::fromMeshBufferToTriangleMesh(hem, mesh_msg.mesh_geometry);

  mesh_msg.header.frame_id = cloud.header.frame_id;
  mesh_msg.header.stamp = cloud.header.stamp;


  std_msgs::ColorRGBA std_color, con_color;
  con_color.r = 1;
  con_color.g = 0.2;
  con_color.b = 0.2;
  con_color.a = 1;
  
  std_color.r = 0.2;
  std_color.g = 1;
  std_color.b = 0.2;
  con_color.a = 1;


  //mesh_msg.mesh_geometry.vertex_colors.resize(mesh_msg.mesh_geometry.vertices.size());
  /*
  std::vector<std_msgs::ColorRGBA> vertex_colors;

  for(int i=0; i<hem.numVertices(); i++){
	vertex_colors[i] = std_color;
  }

  if(fillup_base_hole){
    for(int i=0; i<contour.size();i++){
        vertex_colors[contour[i]] = con_color;
    }
    for(int i=0; i<fillup_indices.size();i++){
      vertex_colors[fillup_indices[i]] = con_color;
    }
  }
*/





  if(success){
	ROS_INFO("Publish organized fast mesh in the %s frame with %d triangles, %d vertices and %d vertex normals", mesh_msg.header.frame_id.c_str(), mesh_msg.mesh_geometry.faces.size(), mesh_msg.mesh_geometry.vertices.size(), mesh_msg.mesh_geometry.vertex_normals.size());
    return true;
  }else{
    ROS_ERROR("conversion from mesh buffer pointer to mesh_msgs::TriangleMeshStamped failed, can not publish organized mesh!");
    return false;
  }
}

bool OrganizedFastMesh::generateOrganizedFastMeshSrv(
  organized_fast_mesh::OrganizedFastMeshSrv::Request& req,
  organized_fast_mesh::OrganizedFastMeshSrv::Response& res)
{

    return generateOrganizedFastMesh(req.organized_scan, res.organized_fast_mesh);
}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
  mesh_msgs::MeshGeometryStamped mesh_msg;
  if(generateOrganizedFastMesh(*cloud, mesh_msg)){
    mesh_pub_.publish(mesh_msg);
  }
}

void print (const mesh_msgs::MeshGeometryStamped msg){
    ROS_INFO("I heard: ");

}

int main(int args, char** argv){
    ros::init(args, argv, "organized_fast_mesh");
    ros::NodeHandle nh;
    OrganizedFastMesh ofm(nh);
    ros::spin();


}
