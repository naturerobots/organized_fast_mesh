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
#include <lvr2/geometry/BaseMesh.hpp>
#include <mesh_msgs/MeshVertexColorsStamped.h>


/**
 * \brief Generates an organized fast mesh out of an organized point cloud
 */
//#TODO delete MeshGenarator
class OrganizedFastMeshGenerator : public lvr2::MeshGenerator<lvr2::ColorVertex < float, int>>

,lvr2::Normal<float>{
    public:

    /**
     * \brief Constrcutor
     * \param organized_scan The organized cloud as lvr2 PointBuffer, height and widht of the cloud
     * to reorganized the cloud (the LVR2 Point Buffer is unorganized but with the knowlegde of height and width you can reorganizde the cloud
     */
    OrganizedFastMeshGenerator(lvr2::PointBuffer
    & cloudBuffer,
    uint32_t heightOfCloud, uint32_t
    widthOfCloud);

    /**
     * TODO delete ??
     */
    void getMesh(lvr2::BaseMesh <lvr2::ColorVertex<float, int>> &mesh);

    /**
     * TODO delete ??
     */
    void getMesh(lvr2::MeshBuffer &mesh, mesh_msgs::MeshVertexColorsStamped &color_msg);

    /**
     * \brief sets the maximum edge length to filter big leaps in the 3D depth data
     */
    void setEdgeThreshold(float dist);

    bool getContour(std::vector<int> &contour_indices);

    void fillContour(std::vector<int> &contour_indices, lvr2::MeshBuffer &mesh, std::vector<int> &fillup_indices);

    private:

    void normalize(int &x, int &y);


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
    bool pointExists(lvr2::BaseVector<float> &vertex);

    //TODO delete ??
    bool pointExists(lvr2::ColorVertex<float, int> &vertex);


    /**
     * \brief checks if the given normal exists
     * \param normal The normal to check for existence
     * \return true if all coords are not nan
     */
    bool normalExists(lvr2::Normal<float> &normal);

    /**
     * \brief tests if the face has long edges
     */
    bool hasLongEdge(int a, int b, int c, float sqr_edge_threshold);

    void showField(std::map<int, int> &index_map, int x, int y);

    bool findContour(std::map<int, int> &index_map, std::vector<int> &contour_indices, int start_x, int start_y, int end_x,
                     int end_y);

    bool isValid(int x, int y);

    bool inBounds(int x, int y);

    /**
     * adds the points and normals of the mesh in to the pointVec and normalVec
     * @param mes
     * @param pointVec
     * @param normalVec
     */
    void lvr2MeshtoStdVector(lvr2::MeshBuffer &mes, std::vector<float> &pointVec, std::vector<float> &normalVec);

    /**
     * adds the points and normals from the mesh in std::vec for a easier handling of the arrays
     * @param mes
     * @param pointVec
     * @param normalVec
     */
    void putStdVectorInMesh(lvr2::MeshBuffer &mes, std::vector<float> &pointVec, std::vector<float> &normalVec);

    /**
     * add faces from a std:vec to the mesh
     * @param mes
     * @param faceVec
     */
    void adFacetoMeshBuffer(lvr2::MeshBuffer &mes, std::vector<int> faceVec);


    //! \holds the organized point cloud
    lvr2::PointBuffer cloudBuffer;
    lvr2::PointBufferPtr mesh_pointsBuffer;
    //! \holds the height and width to reorganized the cloudBuffer
    uint32_t heightOfCloud;
    uint32_t widthOfCloud;


    //! \holds the vertices in the same order as in the mesh
    std::vector <lvr2::ColorVertex<float, int>> vertices;
    //! \threshold value for the longe edge test
    float sqr_edge_threshold;
    //! \map old indices to new indices, invalid indices are -1
    std::map<int, int> index_map;

    size_t index_map_index;


};

#endif /* organized_fast_mesh_generator.h */
