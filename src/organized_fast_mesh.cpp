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
#include <mesh_msgs_conversions/conversions.h>
#include <lvr2/geometry/BaseVector.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <lvr2/geometry/ColorVertex.hpp>
#include <lvr_ros/conversions.h>


typedef lvr2::ColorVertex<float, int> VertexType;
typedef lvr2::Normal<float> NormalType;


OrganizedFastMesh::OrganizedFastMesh(ros::NodeHandle &nh)
        : nh_(nh) {

    cloud_sub_ = nh_.subscribe("/ouster/destaggeredpoints", 20, &OrganizedFastMesh::pointCloud2Callback, this);
    color_pub_ = nh_.advertise<mesh_msgs::MeshVertexColorsStamped>("color", 1);
    mesh_pub_ = nh_.advertise<mesh_msgs::MeshGeometryStamped>("organized_mesh", 1);
    service_ = nh_.advertiseService("organized_fast_mesh", &OrganizedFastMesh::generateOrganizedFastMeshSrv, this);

    ros::NodeHandle p_nh_("~");
    p_nh_.param("edge_threshold", edge_threshold, 0.5);
    p_nh_.param("row_step", row_step, 1);
    p_nh_.param("cal_step", cal_step, 1);



    //set on true to fill up base holes but this feature isn't working in this version
    p_nh_.param("fillup_base_hole", fillup_base_hole, false);

}

bool OrganizedFastMesh::generateOrganizedFastMesh(
        const sensor_msgs::PointCloud2 &cloud, mesh_msgs::MeshGeometryStamped &mesh_msg,
        mesh_msgs::MeshVertexColorsStamped &color_msg) {


    if (cloud.height < 2) {
        ROS_WARN("Received unorganized point cloud!");
        return false;
    }

    //convert from ROS to LVR2
    lvr2::PointBuffer pointBuffer;

    lvr_ros::fromPointCloud2ToPointBuffer(cloud, pointBuffer);
    lvr2::BaseVector<float> left_wheel[8];
    lvr2::BaseVector<float> right_wheel[8];
    left_wheel[0]= lvr2::BaseVector<float>(10,0,-0.8);
    left_wheel[1]= lvr2::BaseVector<float>(-0,0,-0.5);
    left_wheel[2]= lvr2::BaseVector<float>(10,15,-0.8);
    left_wheel[3]= lvr2::BaseVector<float>(-0,15,-0.5);

    left_wheel[4]= lvr2::BaseVector<float>(10,0,-0.8);
    left_wheel[5]= lvr2::BaseVector<float>(-0,0,-0.5);
    left_wheel[6]= lvr2::BaseVector<float>(10,15,-0.8);
    left_wheel[7]= lvr2::BaseVector<float>(-0,15,-0.5);

    right_wheel[0]= lvr2::BaseVector<float>(-0,0,0.8);
    right_wheel[1]= lvr2::BaseVector<float>(10,0,0.5);
    right_wheel[2]= lvr2::BaseVector<float>(0,15,0.8);
    right_wheel[3]= lvr2::BaseVector<float>(10,15,0.5);

    right_wheel[4]= lvr2::BaseVector<float>(-0,0,0.8);
    right_wheel[5]= lvr2::BaseVector<float>(10,0,0.5);
    right_wheel[6]= lvr2::BaseVector<float>(0,15,0.8);
    right_wheel[7]= lvr2::BaseVector<float>(10,15,0.5);

    OrganizedFastMeshGenerator ofmg(pointBuffer, cloud.height, cloud.width,row_step,cal_step,right_wheel,left_wheel);
    ofmg.setEdgeThreshold(edge_threshold);


    lvr2::MeshBufferPtr mesh_buffer_ptr(new lvr2::MeshBuffer);
    ofmg.getMesh(*mesh_buffer_ptr, color_msg);


    std::vector<int> contour, fillup_indices;
    //fillup base is not working
    if (fillup_base_hole) {
        ofmg.getContour(contour);
        ROS_INFO("base hole contour Size: %d", contour.size());
        ofmg.fillContour(contour, *mesh_buffer_ptr, fillup_indices);
    }
    //conversion from LVR2 to ROS Message
    bool success = mesh_msgs_conversions::fromMeshBufferToMeshGeometryMessage(mesh_buffer_ptr, mesh_msg.mesh_geometry);

    //setting the header of the Message
    mesh_msg.header.frame_id = cloud.header.frame_id;
    mesh_msg.header.stamp = cloud.header.stamp;

    //setting colors
    std_msgs::ColorRGBA std_color, con_color;
    con_color.r = 1;
    con_color.g = 0.2;
    con_color.b = 0.2;
    con_color.a = 1;
    std_color.r = 0.2;
    std_color.g = 1;
    std_color.b = 0.2;
    con_color.a = 1;
    color_msg.mesh_vertex_colors.vertex_colors.resize(mesh_msg.mesh_geometry.vertices.size());
    for (int i = 0; i < color_msg.mesh_vertex_colors.vertex_colors.size(); i++) {
        color_msg.mesh_vertex_colors.vertex_colors[i] = std_color;
    }

    //fillup_base isn't working
    if (fillup_base_hole) {
        for (int i = 0; i < contour.size(); i++) {
            color_msg.mesh_vertex_colors.vertex_colors[contour[i]] = con_color;
        }
        for (int i = 0; i < fillup_indices.size(); i++) {
            color_msg.mesh_vertex_colors.vertex_colors[fillup_indices[i]] = con_color;
        }
    }


    if (success) {
        ROS_INFO("Publish organized fast mesh in the %s frame with %d triangles, %d vertices and %d vertex normals",
                 mesh_msg.header.frame_id.c_str(), mesh_msg.mesh_geometry.faces.size(),
                 mesh_msg.mesh_geometry.vertices.size(), mesh_msg.mesh_geometry.vertex_normals.size());



        return true;
    } else {
        ROS_ERROR(
                "conversion from mesh buffer pointer to mesh_msgs::TriangleMeshStamped failed, can not publish organized mesh!");
        return false;
    }
}

bool OrganizedFastMesh::generateOrganizedFastMeshSrv(
        organized_fast_mesh::OrganizedFastMeshSrv::Request &req,
        organized_fast_mesh::OrganizedFastMeshSrv::Response &res) {
    mesh_msgs::MeshVertexColorsStamped color_msg;

    return generateOrganizedFastMesh(req.organized_scan, res.organized_fast_mesh, color_msg);
}

void OrganizedFastMesh::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
    mesh_msgs::MeshGeometryStamped mesh_msg;
    mesh_msgs::MeshVertexColorsStamped color_msg;

    if (generateOrganizedFastMesh(*cloud, mesh_msg, color_msg)) {
        mesh_pub_.publish(mesh_msg);
        color_msg.header = mesh_msg.header;
        color_msg.header.frame_id = mesh_msg.header.frame_id;
        color_pub_.publish(color_msg);
    }
}


int main(int args, char **argv) {
    ros::init(args, argv, "organized_fast_mesh");
    ros::NodeHandle nh;
    OrganizedFastMesh ofm(nh);
    ros::spin();


}
