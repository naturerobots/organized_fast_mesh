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
 *  organized_point_cloud_generation.h
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */


#ifndef ORGANIZED_FAST_MESH_GENERATOR_H_
#define ORGANIZED_FAST_MESH_GENERATOR_H_
#include <lvr2/geometry/HalfEdgeMesh.hpp>

#include <lvr2/reconstruction/MeshGenerator.hpp>
#include <lvr2/geometry/BaseMesh.hpp>
#include <lvr2/geometry/ColorVertex.hpp>
#include <lvr2/geometry/Normal.hpp>
#include <lvr2/geometry/HalfEdgeVertex.hpp>
#include "lvr2/geometry/BaseMesh.hpp"
#include <lvr2/display/PointCloud.hpp>
//TODO Thomas fragen was das gemacht hat
//#include <lvr/geometry/Tesselator.hpp>
#include <lvr2/geometry/BaseMesh.hpp>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <mesh_msgs/MeshVertexColorsStamped.h>


/**
 * \brief Generates an organized fast mesh out of an organized point cloud
 */
//old version

class OrganizedFastMeshGenerator : public lvr2::MeshGenerator<lvr2::ColorVertex<float, int>>,lvr2::Normal<float>{
    public:
    /**
     * \brief Constrcutor
     * \param organized_scan The organized pcl point cloud
     */

   OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointNormal>& organized_scan);
   //OrganizedFastMeshGenerator(lvr2::PointCloud& organized_scan);
    /**
     * \brief generates the organized fast mesh
     * \param a reference to the mesh to fill with the data
     */

    void getMesh(lvr2::BaseMesh<lvr2::ColorVertex<float, int>>& mesh);

    void getMesh(lvr2::HalfEdgeMesh<lvr2::ColorVertex<float, int>>& mesh);

     void getMesh(lvr2::MeshBuffer& mesh,mesh_msgs::MeshVertexColorsStamped& color_msg);
    /**
     * \brief sets the maximum edge length to filter big leaps in the 3D depth data
     */
    void setEdgeThreshold(float dist);

  	bool getContour(std::vector<int>& contour_indices);
    //wahrscheinlich unnötig
    //void fillContour(std::vector<int>& contour_indices, std::shared_ptr<lvr2::MeshBuffer>,std::vector<int>& fillup_indices);

    void fillContour(std::vector<int>& contour_indices, lvr2::MeshBuffer& mesh,std::vector<int>& fillup_indices);

        private:

	    void normalize(int& x, int& y);

    /**
     * \brief converts a pcl point into a lvr color vertex
     * \param in pcl point
     * \param out lvr point

    **/

    void pclToLvrNormal(pcl::PointNormal& in, lvr2::Normal<float>& out);

    /**
     * \brief converts a pcl point into a lvr color vertex
     * \param in pcl point
     * \param out lvr point
     **/

    void pclToLvrVertex(pcl::PointNormal& in, lvr2::ColorVertex<float, int>& out);

    //void pclToLvrVertex(pcl::PointNormal& in, lvr2::ColorVertex<>& out);

    void lvrToPclVertex(const lvr2::ColorVertex<float,int>& vertex, const  lvr2::Normal<float>& normal, pcl::PointNormal& out);


    /**
     * \brief calculates the corresponding index for x and y
     * \param x x-coord
     * \param y y-coord
     * \return index for x and y
     */

    uint32_t toIndex(int x, int y);
    /**
    * \brief checks if the given vertex exists
    * \param vertex The vertex to check for existence
    * \return true if all coords are not nan
    */
    bool pointExists(pcl::PointNormal& point);
    bool pointExists(lvr2::BaseVector<float>& vertex);

    bool pointExists(lvr2::ColorVertex<float,int>& vertex);



    /**
     * \brief checks if the given vertex exists
     * \param vertex The vertex to check for existence
     * \return true if all coords are not nan
     */
   // bool pointExists(lvr2::ColorVertex<float, int>& vertex);

    /**
     * \brief checks if the given normal exists
     * \param normal The normal to check for existence
     * \return true if all coords are not nan
     */
    bool normalExists(lvr2::Normal<float>& normal);

    /**
     * \brief tests if the face has long edges
     */

    long calculateHash(int a, int b, int c,int maxsize);

    bool hasLongEdge(int a, int b, int c, float sqr_edge_threshold);

    void showField(std::map<int, int>& index_map, int x, int y);

    bool findContour(std::map<int, int>& index_map, std::vector<int>& contour_indices, int start_x, int start_y, int end_x, int end_y);

	bool isValid(int x, int y);

	bool inBounds(int x, int y);

    //! \holds the organized point cloud
    pcl::PointCloud<pcl::PointNormal> organized_scan;
    //lvr2::PointCloud organized_scan;
    pcl::PointCloud<pcl::PointNormal>::Ptr mesh_points;
    //lvr2::PointCloud* mesh_points;
    //! \holds the vertices in the same order as in the mesh
    std::vector<lvr2::ColorVertex<float, int> >vertices;
    //! \threshold value for the longe edge test
    float sqr_edge_threshold;
    //! \map old indices to new indices, invalid indices are -1
    std::map<int, int> index_map;

    size_t index_map_index;



};

#endif /* organized_fast_mesh_generator.h */
