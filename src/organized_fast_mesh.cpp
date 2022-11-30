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

/*
    sensor_msgs::PointCloud2 cloud;
    cloud.header = cloudtest.header;

    // describe the bytes
    sensor_msgs::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0 * sizeof(float);
    field_x.datatype = sensor_msgs::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::PointField field_y;
    field_y.name = "y";
    field_y.offset = 1 * sizeof(float);
    field_y.datatype = sensor_msgs::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::PointField field_z;
    field_z.name = "z";
    field_z.offset = 2 * sizeof(float);
    field_z.datatype = sensor_msgs::PointField::FLOAT32;
    field_z.count = 1;

    cloud.fields.push_back(field_x);
    cloud.fields.push_back(field_y);
    cloud.fields.push_back(field_z);

    cloud.point_step = 3 * sizeof(float);
    // insert 5 point in pcl

    cloud.width =3;
    cloud.height = 3;
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data.resize(cloud.row_step *  cloud.height);

    // reinterpret byte memory as float memory
    float* data_raw = reinterpret_cast<float*>(&cloud.data[0]);

    // data_raw[0] = X0
    // data_raw[1] = Y0
    // data_raw[2] = Z0
    // data_raw[3] = X1
    // ...

    data_raw[0] = 1.0;
    data_raw[1] = -1.0;
    data_raw[2] = 0.0;

    data_raw[3] = 1.0;
    data_raw[4] = 1.0;
    data_raw[5] = 0.0;

    data_raw[6] = 0.0;
    data_raw[7] = 0.0;
    data_raw[8] = 0.0;

    data_raw[9] = 0.0;
    data_raw[10] = 0.0;
    data_raw[11] = 0.0;


    data_raw[12] = -1.0;
    data_raw[13] = 1.0;
    data_raw[14] = 0.0;

    data_raw[15] = -1.0;
    data_raw[16] = 2.0;
    data_raw[17] = 0.0;

    data_raw[18] = -3.0;
    data_raw[19] = -1.0;
    data_raw[20] = 0.0;


    data_raw[21] = -3.0;
    data_raw[22] = 1.0;
    data_raw[23] = 0.0;

    data_raw[24] = -3.0;
    data_raw[25] = 2.0;
    data_raw[26] = 0.0;

*/




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

  mesh_msg.header=cloud.header;
  std_msgs::ColorRGBA std_color, con_color;
  con_color.r = 1;
  con_color.g = 0.2;
  con_color.b = 0.2;
  con_color.a = 1;
  
  std_color.r = 0.2;
  std_color.g = 1;
  std_color.b = 0.2;
  con_color.a = 1;
  /*
  mesh_msg.mesh_geometry.vertex_colors.resize(mesh_msg.mesh_geometry.vertices.size());
  
  for(int i=0; i<mesh_msg.mesh_geometry.vertex_colors.size(); i++){
	mesh_msg.mesh_geometry.vertex_colors[i] = std_color;
  }

  if(fillup_base_hole){
    for(int i=0; i<contour.size();i++){
      mesh_msg.mesh_geometry.vertex_colors[contour[i]] = con_color;
    }
    for(int i=0; i<fillup_indices.size();i++){
      mesh_msg.mesh_geometry.vertex_colors[fillup_indices[i]] = con_color;
    }
  }


*/
  /*
    for(int i =0; i<mesh_msg.mesh_geometry.faces.size();i++){
        ROS_INFO("%d", mesh_msg.mesh_geometry.faces[i]);
        if( i%3==0){
            ROS_INFO("new face");

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
