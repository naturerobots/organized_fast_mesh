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
 *  organized_fast_mesh_generator.cpp
 *
 *  author: Sebastian Pütz <spuetz@uni-osnabrueck.de>
 */

#include "organized_fast_mesh_generator.h"
#include <boost/math/special_functions/fpclassify.hpp>

#include <lvr2/io/DataStruct.hpp>
#include <ros/ros.h>
#include <vector>
#include <cmath>


#include <lvr2/registration/KDTree.hpp>


#include <lvr2/reconstruction/SearchTreeFlann.hpp>
#include <lvr2/algorithm/ClusterAlgorithms.hpp>
#include <lvr2/util/ClusterBiMap.hpp>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include "lvr2/geometry/HalfEdge.hpp"
#include "lvr2/attrmaps/AttrMaps.hpp"
#include "lvr2/attrmaps/AttributeMap.hpp"


OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(lvr2::PointBuffer &cloudBuffer , uint32_t heightOfCloud,
                                                       uint32_t widthOfCloud, int row_step, int cal_step  ,  lvr2::BaseVector<float>* right_wheel, lvr2::BaseVector<float>* left_wheel,lvr2::Matrix4<lvr2::BaseVector<float>> matrixTransform)
        : cloudBuffer(cloudBuffer), heightOfCloud(heightOfCloud), widthOfCloud(widthOfCloud) {
    this->row_step=row_step;
    this->cal_step=cal_step;
    this->matrixTransform= matrixTransform;
    this->right_wheel = right_wheel;
    this->left_wheel= left_wheel;
    setEdgeThreshold(0.5);

}


void OrganizedFastMeshGenerator::getMesh(lvr2::MeshBuffer &mesh, mesh_msgs::MeshVertexColorsStamped &color_msg) {
    // clear thePunkt vertices vector
    vertices.clear();

    index_map_index = 0;
    int index_cnt = 0;

    // clear index map
    index_map.clear();

    // add all vertices and normals to the mesh
    // also create an index map for the triangle creation
    std::vector<float> vecPoint;
    std::vector<float> vecNormal;

    lvr2::floatArr cloudPoints = cloudBuffer.getPointArray();
    lvr2::floatArr cloudNormals = cloudBuffer.getNormalArray();
    bool hasColor = false;

    for (int i = 0; i < cloudBuffer.numPoints() * 3; i += 3) {
        int x=(i/3)%widthOfCloud;
        int y= ((i/3)-x)/widthOfCloud;
        if(x%cal_step==0 && y%row_step==0) {

            lvr2::ColorVertex<float, int> point(cloudPoints[i]*matrixTransform[0]+ cloudPoints[i + 1]*matrixTransform[1]+cloudPoints[i + 2]*matrixTransform[2]+matrixTransform[3],cloudPoints[i]*matrixTransform[4]+ cloudPoints[i + 1]*matrixTransform[5]+cloudPoints[i + 2]*matrixTransform[6]+matrixTransform[7],cloudPoints[i]*matrixTransform[8]+ cloudPoints[i + 1]*matrixTransform[9]+cloudPoints[i + 2]*matrixTransform[10]+matrixTransform[11] ); // point at (x,y)

            lvr2::Normal<float> normal;


            if (cloudBuffer.hasNormals()) {
                normal = lvr2::Normal<float>(cloudNormals[i],
                                             cloudNormals[i + 1],
                                             cloudNormals[i + 2]);
            } else {
                normal = lvr2::Normal<float>(0, 1, 0); // normal at (x,y);
            }

            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) ||
                point.z == 0 && point.y == 0 && point.x == 0) {
                // index maps to -1
                index_map[i/3] = -1;
                index_map_index++;
            } else { // if the point exists (not nan)

                if (pointIsPartofMesh(point)) {

                    // index maps to existing vertex in the mesh
                    index_map[i/3] = index_cnt;
                    index_map_index++;
                    index_cnt++;
                    vecPoint.push_back(point.x);
                    vecPoint.push_back(point.y);
                    vecPoint.push_back(point.z);
                    vecNormal.push_back(normal.x);
                    vecNormal.push_back(normal.y);
                    vecNormal.push_back(normal.z);

                    vertices.push_back(point);
                } else {
                    index_map[i/3] = -1;
                    index_map_index++;
                }


            }
        }
        else {
            index_map[i/3] = -1;
            index_map_index++;
        }
    }

    //possible to add possible colors
    color_msg.mesh_vertex_colors.vertex_colors.resize(vecPoint.size() / 3);

    //add points and normals in an array
    boost::shared_array<float> arryPoint(new float[vecPoint.size()]);
    boost::shared_array<float> arryNormal(new float[vecNormal.size()]);
    for (int i = 0; i < vecPoint.size(); i++) {
        arryPoint[i] = vecPoint[i];
        arryNormal[i] = vecNormal[i];
    }
    //add points and normals in to mesh and mesh_points
    mesh.setVertices(arryPoint, vecPoint.size() / 3);
    mesh.setVertexNormals(arryNormal);



    // start adding faces to the mesh
    std::vector<unsigned int> triangleIndexVec;
    for (uint32_t y = 0; y < heightOfCloud; y=y+row_step) {
        for (uint32_t x = 0; x < widthOfCloud; x=cal_step+x) {

            // get indices around the borders for a 360 degree view
            uint32_t x_right = (x >= widthOfCloud-row_step) ? 0 : x + cal_step;
            uint32_t y_bottom = (y >= heightOfCloud-cal_step) ? 0 : y + row_step;

            // get the corresponding indices in the mesh


            int idx = index_map[toIndex(x, y)];

            int idx_r = index_map[toIndex(x_right, y)];
            int idx_rb = index_map[toIndex(x_right, y_bottom)];
            int idx_b = index_map[toIndex(x, y_bottom)];

            //     top              bottom
            //   triangle          triangle
            //    .___.             .   .
            //     \  |             |\
            //      \ |             | \
            //       \|             |  \
            //    .   .             .___.
            //



            // create top triangle if all vertices exists
            if (idx != -1 && idx_rb != -1 && idx_r != -1) {
                // check if there are longer edges then the threshold
                float distance = sqrt(pow(vecPoint[idx],2)+pow(vecPoint[idx+1],2)+pow(vecPoint[idx+2],2));
                if (!hasLongEdge(idx, idx_rb, idx_r, sqr_edge_threshold*(row_step+cal_step)/2)) {


                    triangleIndexVec.push_back(idx);
                    triangleIndexVec.push_back(idx_rb);
                    triangleIndexVec.push_back(idx_r);


                }
            }
            // create bottom triangle if all vertices exists
            if (idx != -1 && idx_b != -1 && idx_rb != -1) {
                // check if there are longer edges then the threshold
                float distance = sqrt(pow(vecPoint[idx],2)+pow(vecPoint[idx+1],2)+pow(vecPoint[idx+2],2));
                if (!hasLongEdge(idx, idx_b, idx_rb, sqr_edge_threshold*(row_step+cal_step)/2)) {


                    triangleIndexVec.push_back(idx);
                    triangleIndexVec.push_back(idx_b);
                    triangleIndexVec.push_back(idx_rb);


                }
            }
        }


    }
    //add faces to the mesh
    boost::shared_array<unsigned int> triangleIndex(new unsigned int[triangleIndexVec.size()]);
    for (int i = 0; i < triangleIndexVec.size(); i++) {
        triangleIndex[i] = triangleIndexVec[i];
    }
    mesh.setFaceIndices(triangleIndex, triangleIndexVec.size() / 3);
}


bool OrganizedFastMeshGenerator::getContour(std::vector<int> &contour_indices) {
    // fill up robot shadow in the mesh
    bool found_contour = false;

    int height = heightOfCloud;
    int width = widthOfCloud;

    int start_x, start_y, end_x, end_y;

    for (int x = width - 1; x > 0; x--) {
        if (index_map[widthOfCloud * x] != -1) {
            start_x = x;
            start_y = 0;
            break;
        }
    }
    start_x = 0;
    start_y = 0;


    for (int x = width - 1; x > 0 && !found_contour; x--) {
        if (index_map[widthOfCloud * x + height - 1] != -1) {
            end_x = x;
            end_y = height - 1;
            break;
        }
    }


    std::cout << "start_x: " << start_x << std::endl;
    std::cout << "start_y: " << start_y << std::endl;
    std::cout << "end_x: " << end_x << std::endl;
    std::cout << "end_y: " << end_y << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;


    return findContour(index_map, contour_indices, start_x, start_y, start_x, start_y);

}

void OrganizedFastMeshGenerator::setEdgeThreshold(float dist) {
    sqr_edge_threshold = dist * dist;
}

void OrganizedFastMeshGenerator::showField(std::map<int, int> &index_map, int x, int y) {
    bool a = index_map[toIndex(x - 1, y + 1)] != -1;
    bool b = index_map[toIndex(x, y + 1)] != -1;
    bool c = index_map[toIndex(x + 1, y + 1)] != -1;

    bool d = index_map[toIndex(x - 1, y)] != -1;
    bool e = index_map[toIndex(x, y)] != -1;
    bool f = index_map[toIndex(x + 1, y)] != -1;

    bool g = index_map[toIndex(x - 1, y - 1)] != -1;
    bool h = index_map[toIndex(x, y - 1)] != -1;
    bool i = index_map[toIndex(x + 1, y - 1)] != -1;

    std::cout << (a ? "#" : "O") << (b ? "#" : "O") << (c ? "#" : "O") << std::endl;
    std::cout << (d ? "#" : "O") << (e ? "#" : "O") << (f ? "#" : "O") << std::endl;
    std::cout << (g ? "#" : "O") << (h ? "#" : "O") << (i ? "#" : "O") << std::endl;

}


bool
OrganizedFastMeshGenerator::findContour(std::map<int, int> &index_map, std::vector<int> &contour_indices, int start_x,
                                        int start_y, int end_x, int end_y) {
    //algrithem 3
    contour_indices.clear();

    int height = heightOfCloud;
    int width = widthOfCloud;

    int x = start_x;
    int y = start_y;
    ROS_INFO("%d", index_map.size());
    int index = index_map[x * width + y];
    contour_indices.push_back(index);

    if (index == -1) {
        std::cerr << "return false: invalid (x,y): (" << x << ", " << y << ")" << std::endl;
        return false;
    }

    bool ri = isValid(x, y + 1);
    bool li = isValid(x, y - 1);
    bool bi = isValid(x + 1, y);
    bool fi = isValid(x - 1, y);

    int px = x;
    int py = y;

    if (!ri && fi) x--;
    else if (!fi && li) y--;
    else if (!li && bi) x++;
    else if (!bi && ri) y++;
    else return false;

    while (x != end_x || y != end_y) {

        index = index_map[x * width + y];
        contour_indices.push_back(index);

        int dx = 0;
        int dy = 0;

        if (px == 0 && x == width - 1) {
            dx = -1;
        } else if (px == width - 1 && x == 0) {
            dx = +1;
        } else {
            dx = x - px;
        }

        if (py == 0 && y == height - 1) {
            dy = -1;
        } else if (py == height - 1 && y == 0) {
            dy = +1;
        } else {
            dy = y - py;
        }


        // transpose d
        bool ri = isValid(x + dy, y - dx);
        bool li = isValid(x - dy, y + dx);
        bool fi = isValid(x + dx, y + dy);
        bool pri = isValid(px + dy, py - dx);

        int next_x, next_y;
        if (std::abs(dx) + std::abs(dy) != 1) {
            std::cout << "Error:  dx: " << dx << " dy: " << dy << std::endl;
            return false;
        }

        if (ri) {
            next_x = x + dy;
            next_y = y - dx;
        } else if (fi) {
            next_x = x + dx;
            next_y = y + dy;
        } else if (li) {
            next_x = x - dy;
            next_y = y + dx;
        } else {
            next_x = x - dx;
            next_y = y - dy;
        }

        px = x;
        py = y;
        x = next_x;
        y = next_y;
        normalize(x, y);
    }

}


inline bool OrganizedFastMeshGenerator::isValid(int x, int y) {
    const int height = heightOfCloud;
    const int width = widthOfCloud;

    bool around_the_corner_x = false;
    bool around_the_corner_y = true;

    if (around_the_corner_x) {
        while (x >= width)
            x -= width;
        while (y < 0)
            y += width;
    }

    if (around_the_corner_y) {
        while (y >= height)
            y -= height;
        while (y < 0)
            y += height;
    }

    return inBounds(x, y) && index_map[toIndex(x, y)] != -1;
}

inline bool OrganizedFastMeshGenerator::inBounds(int x, int y) {
    const int height = heightOfCloud;
    const int width = widthOfCloud;;

    return
            0 <= x && x < width && 0 <= y && y < height;
}

inline bool OrganizedFastMeshGenerator::hasLongEdge(int a, int b, int c, float sqr_edge_threshold) {
    lvr2::ColorVertex<float, int> v_a = vertices[a];
    lvr2::ColorVertex<float, int> v_b = vertices[b];
    lvr2::ColorVertex<float, int> v_c = vertices[c];

    if (v_a.squaredDistanceFrom(v_b) > sqr_edge_threshold) return true;
    if (v_b.squaredDistanceFrom(v_c) > sqr_edge_threshold) return true;
    if (v_c.squaredDistanceFrom(v_a) > sqr_edge_threshold) return true;
    return false;
}


inline uint32_t OrganizedFastMeshGenerator::toIndex(int x, int y) {
    uint32_t height = heightOfCloud;
    uint32_t width = widthOfCloud;

    //normalize(x, y);
    //return x*height+y;
    return y * width + x;
}



inline void OrganizedFastMeshGenerator::normalize(int &x, int &y) {
    uint32_t height = heightOfCloud;
    uint32_t width = widthOfCloud;

    bool around_the_corner_x = false;
    bool around_the_corner_y = true;

    // modulo to stay in bounds
    if (around_the_corner_x) {
        while (x >= width) x -= width;
        while (x < 0) x += width;
    }
    if (around_the_corner_y) {
        while (y >= height) y -= height;
        while (y < 0) y += height;
    }
}

inline void OrganizedFastMeshGenerator::lvr2MeshtoStdVector(lvr2::MeshBuffer &mesh, std::vector<float> &pointVec,
                                                            std::vector<float> &normalVec) {
    lvr2::floatArr pointArr = mesh.getVertices();
    lvr2::floatArr normalArr = mesh.getVertexNormals();

    for (int i = 0; i < mesh.numVertices() * 3; i++) {
        pointVec.push_back(pointArr[i]);
        normalVec.push_back(pointArr[i]);


    }
}

inline void OrganizedFastMeshGenerator::putStdVectorInMesh(lvr2::MeshBuffer &mesh, std::vector<float> &pointVec,
                                                           std::vector<float> &normalVec) {
    lvr2::floatArr pointArr(new float(pointVec.size()));
    lvr2::floatArr normalArr(new float(normalVec.size()));
    for (int i = 0; i < pointVec.size(); i++) {
        pointArr[i] = pointVec[i];
    }

    for (int i = 0; i < normalVec.size(); i++) {
        normalArr[i] = normalVec[i];
    }

    mesh.setVertices(pointArr, pointVec.size() / 3);
    mesh.setVertices(normalArr, normalVec.size() / 3);


}

inline void OrganizedFastMeshGenerator::adFacetoMeshBuffer(lvr2::MeshBuffer &mesh, std::vector<int> faceVec) {
    lvr2::indexArray face(new unsigned int(mesh.numFaces() + faceVec.size()));
    face = mesh.getFaceIndices();
    for (int i = mesh.numFaces(); i < mesh.numFaces() + faceVec.size(); i++) {
        face[i] = faceVec[i - mesh.numFaces()];
    }
    mesh.setFaceIndices(face, mesh.numFaces() + faceVec.size());

}


void OrganizedFastMeshGenerator::fillContour(std::vector<int> &contour_indices, lvr2::MeshBuffer &mesh,
                                             std::vector<int> &fillup_indices) {
    //NOT WORKING !!!!
    std::vector<int>::iterator c_iter;
    std::map<int, int> hole_index_map;
    //get the Mesh points as vector
    std::vector<float> pointVec;
    std::vector<float> normalVec;
    lvr2MeshtoStdVector(mesh, pointVec, normalVec);

    std::cout << "Remove directly duplicate indices in contour..." << std::endl;
    // remove consecutive duplicates
    std::vector<int> clean_indices = contour_indices;
    bool clean;
    do {
        clean = true;
        std::vector<int> tmp_indices;
        int a, b;
        for (int i = 0; i < clean_indices.size() - 2; i++) {


            a = clean_indices[i];
            b = clean_indices[i + 2];
            tmp_indices.push_back(a);

            if (a == b) {
                i += 2;
                clean = false;
            }
        }
        if (a != b) {
            tmp_indices.push_back(*(clean_indices.end() - 2));
            tmp_indices.push_back(*(clean_indices.end() - 1));
        }
        clean_indices = tmp_indices;
    } while (!clean);

    std::cout << "Removed " << contour_indices.size() - clean_indices.size() << " duplicate indices of "
              << contour_indices.size() << " indices in the contour." << std::endl;
    contour_indices = clean_indices;
    for (int i = 0; i < contour_indices.size(); i++) {
        if (contour_indices[i] == -1) {
            std::cerr << "Error: Invalid point in contour at index " << i << "!" << std::endl;
            return;
        }
    }

    // calculate the centroid of the contour
    lvr2::BaseVector<float> centroid;
    for (c_iter = contour_indices.begin();
         c_iter != contour_indices.end(); ++c_iter) {
        lvr2::BaseVector<float> current_vertex(mesh.getVertices()[*c_iter],
                                               mesh.getVertices()[*c_iter + 1],
                                               mesh.getVertices()[*c_iter + 2]);

        centroid += current_vertex;
    }
    centroid /= contour_indices.size();
    lvr2::Normal<float> normal(0, 0, 1);


    int num_sub_contours = (int) floor(log2(contour_indices.size()));


    std::vector<std::vector<int>> hole_triangles;
    std::vector<std::vector<int>> hole_indices;

    hole_indices.push_back(contour_indices);
    int skip = 1;

    for (int i = 0; i < num_sub_contours; i++) {
        std::vector<int> inner_contour;
        float scale = 1 - (float) (i + 1) / num_sub_contours;
        skip *= 2;
        for (int j = 0; j < contour_indices.size(); j += skip) {

            lvr2::BaseVector<float> con_vertex(vertices[contour_indices[j]]);

            //add con_vertex to the vector how holds the meshpoints
            pointVec.push_back(centroid[0] + ((con_vertex[0]) - centroid[0] * scale));
            pointVec.push_back(centroid[1] + ((con_vertex[1]) - centroid[2] * scale));
            pointVec.push_back(centroid[2] + ((con_vertex[2]) - centroid[2] * scale));
            normalVec.push_back(0);
            normalVec.push_back(0);
            normalVec.push_back(1);

            //pointVec/3 = mesh_pointBuffer->size after adding all new points
            int new_index = ((int) (pointVec.size() / 3) - 1);
            fillup_indices.push_back(new_index);
            inner_contour.push_back(new_index);
        }
        hole_indices.push_back(inner_contour);
    }
    //add centerroid to mesh
    pointVec.push_back(centroid[0]);
    pointVec.push_back(centroid[1]);
    pointVec.push_back(centroid[2]);
    normalVec.push_back(0);
    normalVec.push_back(0);
    normalVec.push_back(1);


    std::vector<int> inner_contour;
    int new_index = ((int) pointVec.size() / 3) - 1;
    fillup_indices.push_back(new_index);
    inner_contour.push_back(new_index);
    hole_indices.push_back(inner_contour);
    //algrithem 4
    for (int j = 0; j < hole_indices.size() - 3; j++) {
        int n_j0 = hole_indices[j + 0].size();
        int n_j1 = hole_indices[j + 1].size();


        for (int i = 0; i < n_j0 + 1; i++) {
            int a = hole_indices[j][i % n_j0];
            int b = hole_indices[j + 1][(i / 2) % n_j1];
            int c = hole_indices[j][(i + 1) % n_j0];
            int d = hole_indices[j + 1][(i + 1) / 2 % n_j1];

            if (i % 2 == 0) {
                std::vector<int> t1(3, 0);
                t1[0] = c;
                t1[1] = b;
                t1[2] = a;
                hole_triangles.push_back(t1);
            } else {
                std::vector<int> t2(3, 0);
                std::vector<int> t3(3, 0);

                t2[0] = a;
                t2[1] = d;
                t2[2] = b;

                t3[0] = c;
                t3[1] = d;
                t3[2] = a;
                hole_triangles.push_back(t2);
                hole_triangles.push_back(t3);
            }
        }
    }


    int center_index = hole_indices[hole_indices.size() - 1][0];
    int j_sub = hole_indices.size() - 2;
    int n_j_sub = hole_indices[j_sub].size();
    for (int i = 0; i < hole_indices[j_sub].size(); i++) {
        int a = hole_indices[j_sub][(i + 1) % n_j_sub];
        int b = hole_indices[j_sub][i];
        int c = center_index;

        std::vector<int> t4(3, 0);
        t4[0] = a;
        t4[1] = b;
        t4[2] = c;
        hole_triangles.push_back(t4);
    }
    //add all points and normals to mesh
    lvr2::floatArr pointArr(new float(pointVec.size()));
    lvr2::floatArr normalArr(new float(normalVec.size()));
    for (int i = 0; i < pointVec.size(); i++) {
        pointArr[i] = pointVec[i];
    }
    for (int i = 0; i < normalVec.size(); i++) {
        normalArr[i] = normalVec[i];
    }

    mesh.setVertices(pointArr, pointVec.size() / 3);
    mesh.setVertexNormals(normalArr);
    /*

    //why ??
pcl::PointIndices::Ptr in_radius_indices (new pcl::PointIndices);

  std::vector<float> radius_distances;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(mesh_points);
  tree->radiusSearch(pcl_centroid, 1.2, in_radius_indices->indices, radius_distances);

  std::cerr << "fitting a plane onto the hole vertices... " << std::endl;
  std::cout << "number of in-radius indices: " << in_radius_indices->indices.size() << std::endl;
  std::cout << "size of mesh-points: " << mesh_points->size() << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointNormal> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (mesh_points);
  seg.setIndices(in_radius_indices);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given hole vertices dataset.");
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    << coefficients->values[1] << " "
    << coefficients->values[2] << " "
    << coefficients->values[3] << std::endl;


  std::cerr << "Number of Inliers: " << inliers->indices.size () << std::endl;
  std::cerr << "Size of mesh points: " << mesh_points->size() << std::endl;

  lvr::Normalf hole_normal(
      coefficients->values[0],
      coefficients->values[1],
      coefficients->values[2]);

  pcl::ProjectInliers<pcl::PointNormal> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (mesh_points);
  proj.setIndices(inliers);
  proj.setCopyAllData(true);
  proj.setModelCoefficients (coefficients);
  proj.filter (*mesh_points);

  /*
     pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
     mls.setComputeNormals (false);
     mls.setInputCloud (pcl_cloud);
     mls.setIndices(radius_indices);
     mls.setPolynomialFit (true);
     mls.setSearchMethod (tree);
     mls.setSearchRadius (0.3);
     mls.process(*mls_points);
   */
    /*
    lvr::HalfEdgeMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >* h_mesh;
    h_mesh = static_cast<lvr::HalfEdgeMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >* >(&mesh);
    std::vector<lvr::HalfEdgeVertex<lvr::ColorVertex<float, int>, lvr::Normal<float> >* >& mesh_vertices = h_mesh->getVertices();

    for(int i=0; i< inliers->indices.size(); i++){
        int index = inliers->indices[i];
        mesh_vertices[index]->m_position.x = (*mesh_points)[index].x;
        mesh_vertices[index]->m_position.y = (*mesh_points)[index].y;
        mesh_vertices[index]->m_position.z = (*mesh_points)[index].z;
        mesh_vertices[index]->m_normal.x = hole_normal.x;
        mesh_vertices[index]->m_normal.y = hole_normal.y;
        mesh_vertices[index]->m_normal.z = hole_normal.z;

        if(!pointExists(mesh_vertices[index]->m_position) || !normalExists(mesh_vertices[index]->m_normal)){
            std::cout << "invalid point or normal with buffer index: " << index <<  std::endl;
        }
    }

    */
    //adding the new faces the mesh
    std::vector<unsigned int> newFaces;
    lvr2::indexArray oldFaceArr = mesh.getFaceIndices();
    for (int i = 0; i < mesh.numFaces(); i++) {
        newFaces.push_back(oldFaceArr[i]);
    }

    for (size_t i = 0; i < hole_triangles.size(); i++) {
        if (hole_triangles[i].size() != 3) {
            std::cerr << "wrong number of triangle indices, should be three! -- triangle index: " << i << std::endl;
            continue;
        }
        int a = hole_triangles[i][0];
        int b = hole_triangles[i][1];
        int c = hole_triangles[i][2];

        if (a < 0 || a >= mesh.size()) {
            std::cerr << "invalid index:" << a << std::endl;
            continue;
        }
        if (b < 0 || b >= mesh.size()) {
            std::cerr << "invalid index:" << b << std::endl;
            continue;
        }
        if (c < 0 || c >= mesh.size()) {
            std::cerr << "invalid index:" << c << std::endl;
            continue;
        }
        newFaces.push_back(hole_triangles[i][0]);
        newFaces.push_back(hole_triangles[i][1]);
        newFaces.push_back(hole_triangles[i][2]);
    }
    lvr2::indexArray newFaceArr(new unsigned int(newFaces.size()));
    for (int i = 0; i < newFaces.size();) {
        newFaceArr[i] = newFaces[i];
    }
    mesh.setFaceIndices(newFaceArr, newFaces.size() / 3);
}

bool OrganizedFastMeshGenerator::pointIsPartofMesh(lvr2::ColorVertex<float, int> point){
    if(right_wheel == nullptr && left_wheel ==nullptr){
        return true;
    }else{
        if(right_wheel != nullptr && left_wheel !=nullptr) {
            return isInsideBox(point, right_wheel) || isInsideBox(point, left_wheel);
        }
        else if(right_wheel != nullptr ){
            return isInsideBox(point, right_wheel);
        }
        else if(left_wheel != nullptr ){
            return isInsideBox(point, left_wheel);
        }


    }
}

bool OrganizedFastMeshGenerator::isInsideBox(lvr2::BaseVector<float> p, lvr2::BaseVector<float>* vertices) {
    // Berechne die minimalen und maximalen Grenzwerte der Koordinaten des Quaders
    lvr2::BaseVector<float> bMin, bMax;
    bMin.x = bMax.x = vertices[0].x;
    bMin.y = bMax.y = vertices[0].y;
    bMin.z = bMax.z = vertices[0].z;
    for (int i = 1; i < 8; i++) {
        if (vertices[i].x < bMin.x) bMin.x = vertices[i].x;
        if (vertices[i].x > bMax.x) bMax.x = vertices[i].x;
        if (vertices[i].y < bMin.y) bMin.y = vertices[i].y;
        if (vertices[i].y > bMax.y) bMax.y = vertices[i].y;
        if (vertices[i].z < bMin.z) bMin.z = vertices[i].z;
        if (vertices[i].z > bMax.z) bMax.z = vertices[i].z;
    }

    // Überprüfe, ob der Punkt innerhalb des Quaders liegt
    if (p.x < bMin.x || p.x > bMax.x ||
        p.y < bMin.y || p.y > bMax.y ||
        p.z < bMin.z || p.z > bMax.z)
        return false;
    else
        return true;
}

