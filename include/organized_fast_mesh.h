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
 *  organized_fast_mesh.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#ifndef ORGANIZED_FAST_MESH_H_
#define ORGANIZED_FAST_MESH_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "organized_fast_mesh/OrganizedFastMeshSrv.h"
#include <mesh_msgs/MeshTriangleIndices.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/MeshVertexColorsStamped.h>

#include <std_msgs/ColorRGBA.h>

class OrganizedFastMesh {

public:
    OrganizedFastMesh(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher mesh_pub_;
    ros::Publisher color_pub_;
    ros::ServiceServer service_;

    void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &cloud);

    bool generateOrganizedFastMeshSrv(
            organized_fast_mesh::OrganizedFastMeshSrv::Request &req,
            organized_fast_mesh::OrganizedFastMeshSrv::Response &res);

    bool generateOrganizedFastMesh(
            const sensor_msgs::PointCloud2 &cloud,
            mesh_msgs::MeshGeometryStamped &mesh_msg, mesh_msgs::MeshVertexColorsStamped &color_msg);

    double edge_threshold;
    int cal_step;
    int row_step;
    bool fillup_base_hole;
    float left_wheel;
    float right_wheel;
    float delta;
    float min_x;
    float max_z;

};

#endif /* organized_fast_mesh.h */