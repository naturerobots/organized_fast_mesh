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
#include <math.h>
#include <pcl-1.10/pcl/sample_consensus/method_types.h>
#include <pcl-1.10/pcl/sample_consensus/model_types.h>
#include <pcl-1.10/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.10/pcl/search/search.h>
#include <pcl-1.10/pcl/filters/project_inliers.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <pcl-1.10/pcl/point_types.h>
#include <pcl-1.10/pcl/search/kdtree.h>
#include <lvr2/registration/KDTree.hpp>
#include <pcl-1.10/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.10/pcl/surface/marching_cubes_rbf.h>
#include <pcl-1.10/pcl/ModelCoefficients.h>
#include <pcl-1.10/pcl/PointIndices.h>
#include <lvr2/registration/KDTree.hpp>
#include <lvr2/reconstruction/SearchTree.hpp>


OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(lvr2::PointBuffer& cloudBuffer,  uint32_t heightOfCloud, uint32_t widthOfCloud)
: cloudBuffer(cloudBuffer), heightOfCloud(heightOfCloud), widthOfCloud(widthOfCloud)
{
    setEdgeThreshold(0.5);

}




void getMesh(lvr2::HalfEdgeMesh<lvr2::ColorVertex<float, int>>& mesh){
    //TODO wie unten wenn möglich
}

void OrganizedFastMeshGenerator::getMesh(lvr2::BaseMesh<lvr2::ColorVertex<float, int>>& mesh) {
}

void OrganizedFastMeshGenerator::getMesh(lvr2::MeshBuffer& mesh,mesh_msgs::MeshVertexColorsStamped& color_msg) {
    // clear the vertices vector
    vertices.clear();


    //mesh_points = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);

    mesh_pointsBuffer =lvr2::PointBuffer();

    std::vector<char> colors;
    index_map_index = 0;
    int index_cnt = 0;

    // clear index map
    index_map.clear();

    // add all vertices and normals to the mesh
    // also create an index map for the triangle creation
    std::vector<float> vecPoint;
    std::vector<float> vecNormal;

    lvr2::floatArr cloudPoints = cloudBuffer.getPointArray();
    lvr2::floatArr cloudNormals =cloudBuffer.getNormalArray();
    bool hasColor=false;







        for(int i=0; i<cloudBuffer.numPoints()*3;i+=3){


        lvr2::ColorVertex<float, int> point(cloudPoints[i],cloudPoints[i+1],cloudPoints[i+2]); // point at (x,y)
        lvr2::Normal<float> normal;


            if(cloudBuffer.hasNormals()){
             normal = lvr2::Normal<float>(cloudNormals[(x*widthOfCloud+y)*3+0],cloudNormals[x*widthOfCloud+y+1],cloudNormals[(x*widthOfCloud+y)*3+2]);
            }else {
                //ANSPRECHEN
                 normal=lvr2::Normal<float> (0, 1, 0); // normal at (x,y);
            }

            if (std::isnan(point.x) ||std::isnan(point.y)||std::isnan(point.z) || point.z==0 &&  point.y==0 && point.x==0 ) {
                // index maps to -1
                index_map[index_map_index] = -1;
                index_map_index++;
            } else { // if the point exists (not nan)
                // index maps to existing vertex in the mesh
                index_map[index_map_index] = index_cnt;
                index_map_index++;
                index_cnt++;
                vecPoint.push_back(point.x);
                vecPoint.push_back(point.y);
                vecPoint.push_back(point.z);
                vecNormal.push_back(normal.x);
                vecNormal.push_back(normal.y);
                vecNormal.push_back(normal.z);

                vertices.push_back(point);







            }
        }




    color_msg.mesh_vertex_colors.vertex_colors.resize(vecPoint.size()/3);



    boost::shared_array<float> arryPoint(new float[vecPoint.size()]);
    boost::shared_array<float> arryNormal(new float[vecNormal.size()]);
    for (int i=0;i<vecPoint.size();i++){
        arryPoint[i]=vecPoint[i];
        arryNormal[i]=vecNormal[i];




    }
/*
    for (int i=0;i<vecPoint.size()/3;i++){
        color_msg.mesh_vertex_colors.vertex_colors[i].a=colors[i*4+0];
        color_msg.mesh_vertex_colors.vertex_colors[i].r=colors[i*4+1];
        color_msg.mesh_vertex_colors.vertex_colors[i].g=colors[i*4+2];
        color_msg.mesh_vertex_colors.vertex_colors[i].b=colors[i*4+3];
    }
*/
    mesh.setVertices(arryPoint,vecPoint.size()/3 );
    mesh.setVertexNormals(arryNormal);
    mesh_pointsBuffer. setPointArray(arryPoint,vecPoint.size()/3 );
    mesh_pointsBuffer.setNormalArray(arryNormal,vecPoint.size()/3 );







    // start adding faces to the mesh
    std::vector<unsigned int> triangleIndexVec;
    for(uint32_t y=0; y<heightOfCloud; y++){
        for(uint32_t x=0; x<widthOfCloud; x++){

            // get indices around the borders for a 360 degree view
            uint32_t x_right = (x == widthOfCloud) ? 0 : x+1;
            uint32_t y_bottom = (y == heightOfCloud) ? 0 : y+1;

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
            if (idx != -1 && idx_rb  != -1 && idx_r  != -1) {
                // check if there are longer edges then the threshold
                if (!hasLongEdge(idx, idx_rb, idx_r, sqr_edge_threshold)) {


                            triangleIndexVec.push_back(idx);
                            triangleIndexVec.push_back(idx_rb);
                            triangleIndexVec.push_back(idx_r);


                }
            }
                // create bottom triangle if all vertices exists
                if (idx  != -1 && idx_b  != -1 && idx_rb  != -1 ) {
                    // check if there are longer edges then the threshold
                    if (!hasLongEdge(idx, idx_b, idx_rb, sqr_edge_threshold)) {



                                triangleIndexVec.push_back(idx);
                                triangleIndexVec.push_back(idx_b);
                                triangleIndexVec.push_back(idx_rb);


                    }
                }
            }


        }

    boost::shared_array<unsigned int> triangleIndex(new unsigned int[triangleIndexVec.size()]);

    for(int i =0;i<triangleIndexVec.size();i++){

        triangleIndex[i]=triangleIndexVec[i];

    }

    mesh.setFaceIndices(triangleIndex,triangleIndexVec.size()/3);
}



bool OrganizedFastMeshGenerator::getContour(std::vector<int>& contour_indices){
  // fill up robot shadow in the mesh
  bool found_contour = false;

  int height = heightOfCloud;
  int width = widthOfCloud;

  int start_x, start_y, end_x, end_y;
  for(int x=width-1; x > 0; x--){
    if(index_map[toIndex(x, 0)] != -1){
      start_x = x;
      start_y = 0;
      break;
    }
  }

  for(int x=width-1; x > 0 && !found_contour; x--){
    if(index_map[toIndex(x, height-1)] != -1){
      end_x = x;
      end_y = height-1;
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

void OrganizedFastMeshGenerator::setEdgeThreshold(float dist){
  sqr_edge_threshold = dist * dist;
}

void OrganizedFastMeshGenerator::showField(std::map<int, int>& index_map, int x, int y){
  bool a = index_map[toIndex(x-1, y+1)] != -1;
  bool b = index_map[toIndex(x,   y+1)] != -1;
  bool c = index_map[toIndex(x+1, y+1)] != -1;

  bool d = index_map[toIndex(x-1, y)] != -1;
  bool e = index_map[toIndex(x,   y)] != -1;
  bool f = index_map[toIndex(x+1, y)] != -1;

  bool g = index_map[toIndex(x-1, y-1)] != -1;
  bool h = index_map[toIndex(x,   y-1)] != -1;
  bool i = index_map[toIndex(x+1, y-1)] != -1;

  std::cout << (a ? "#" : "O") << (b ? "#" : "O") << (c ? "#" : "O") << std::endl;
  std::cout << (d ? "#" : "O") << (e ? "#" : "O") << (f ? "#" : "O") << std::endl;
  std::cout << (g ? "#" : "O") << (h ? "#" : "O") << (i ? "#" : "O") << std::endl;

}

bool OrganizedFastMeshGenerator::findContour(std::map<int, int>& index_map, std::vector<int>& contour_indices, int start_x, int start_y, int end_x, int end_y){
  contour_indices.clear();

  int height = heightOfCloud;
  int width = widthOfCloud;

  int x = start_x;
  int y = start_y;

  uint32_t index = index_map[toIndex(x, y)];
  contour_indices.push_back(index);

  if(index == -1){
    std::cerr << "return false: invalid (x,y): (" << x << ", "<< y << ")" << std::endl;
    return false;
  }

  bool ri = isValid(x, y+1);
  bool li = isValid(x, y-1);
  bool bi = isValid(x+1, y);
  bool fi = isValid(x-1, y);

  int px = x;
  int py = y;

  if(!ri && fi)       x--;
  else if(!fi && li)  y--;
  else if(!li && bi)  x++;
  else if(!bi && ri)  y++;
  else return false;

  while(x != end_x || y != end_y){

    index = index_map[toIndex(x, y)];
    contour_indices.push_back(index);

    int dx = 0;
    int dy = 0;

    if(px == 0 && x == width-1){
      dx = -1;
    }else if(px == width-1 && x == 0){
      dx = +1;
    }else{
      dx = x-px;
    }

    if(py == 0 && y == height-1){
      dy = -1;
    }else if(py == height-1 && y == 0){
      dy = +1;
    }else{
      dy = y-py;
    }


    // transpose d
    bool ri = isValid(x+dy, y-dx);
    bool li = isValid(x-dy, y+dx);
    bool fi = isValid(x+dx, y+dy);
    bool pri = isValid(px+dy, py-dx);

    int next_x, next_y;

    if(std::abs(dx) + std::abs(dy) != 1){
      std::cout << "Error:  dx: " << dx << " dy: " << dy << std::endl;
      return false;
    }

    if(ri){
      next_x = x+dy;
      next_y = y-dx;
    }else if(fi){
      next_x = x+dx;
      next_y = y+dy;
    }else if(li){
      next_x = x-dy;
      next_y = y+dx;
    }else{
      next_x = x-dx;
      next_y = y-dy;
    }

    px = x;
    py = y;
    x = next_x;
    y = next_y;
    normalize(x, y);
  }
}



long OrganizedFastMeshGenerator::calculateHash(int a, int b, int c,int maxsize){

    int arr[] ={a,b,c};
    int n = sizeof(arr) / sizeof(arr[0]);
    sort(arr,arr+n);
    int big=arr[2];
    int middel=arr[1];
    int small=arr[0];

    return big * pow(maxsize,2) + middel* maxsize+small;

}


inline bool OrganizedFastMeshGenerator::isValid(int x, int y){
  const int height =heightOfCloud;
  const int width = widthOfCloud;

  bool around_the_corner_x = false;
  bool around_the_corner_y = true;

  if(around_the_corner_x){
    while(x >= width)
      x -= width;
    while(y < 0)
      y += width;
  }

  if(around_the_corner_y){
    while(y >= height)
      y -= height;
    while(y < 0)
      y += height;
  }

  return inBounds(x, y) && index_map[toIndex(x, y)] != -1;
}

inline bool OrganizedFastMeshGenerator::inBounds(int x, int y){
  const int height = heightOfCloud;
  const int width = widthOfCloud;;

  return
    0 <= x && x < width && 0 <= y && y < height;
}

inline bool OrganizedFastMeshGenerator::hasLongEdge(int a, int b, int c, float sqr_edge_threshold){
  lvr2::ColorVertex<float, int> v_a = vertices[a];
  lvr2::ColorVertex<float, int> v_b = vertices[b];
  lvr2::ColorVertex<float, int> v_c = vertices[c];

  if(v_a.squaredDistanceFrom(v_b) > sqr_edge_threshold) return true;
  if(v_b.squaredDistanceFrom(v_c) > sqr_edge_threshold) return true;
  if(v_c.squaredDistanceFrom(v_a) > sqr_edge_threshold) return true;
  return false;
}


inline uint32_t OrganizedFastMeshGenerator::toIndex(int x, int y){
  uint32_t height = heightOfCloud;
  uint32_t width = widthOfCloud;

  //normalize(x, y);
  //return x*height+y;
  return y*width+x;
}

inline bool OrganizedFastMeshGenerator::pointExists(lvr2::ColorVertex<float, int>& vertex){
  return
    boost::math::isfinite<float>(vertex.x) &&
    boost::math::isfinite<float>(vertex.y) &&
    boost::math::isfinite<float>(vertex.z);
}

inline bool OrganizedFastMeshGenerator::pointExists(lvr2::BaseVector<float>& vertex){
    return
            boost::math::isfinite<float>(vertex.x) &&
            boost::math::isfinite<float>(vertex.y) &&
            boost::math::isfinite<float>(vertex.z);
}


inline bool OrganizedFastMeshGenerator::normalExists(lvr2::Normal<float>& normal){
  return
    boost::math::isfinite<float>(normal.x) &&
    boost::math::isfinite<float>(normal.y) &&
    boost::math::isfinite<float>(normal.z);
}



inline void OrganizedFastMeshGenerator::normalize(int& x, int& y){
  uint32_t height = heightOfCloud;
  uint32_t width = widthOfCloud;

  bool around_the_corner_x = false;
  bool around_the_corner_y = true;

  // modulo to stay in bounds
  if(around_the_corner_x){
    while(x >= width) x -= width;
    while(x < 0) x += width;
  }
  if(around_the_corner_y){
    while(y >= height) y -= height;
    while(y < 0) y += height;
  }
}
inline void OrganizedFastMeshGenerator::lvr2MeshtoStdVector(lvr2::MeshBuffer& mesh, std::vector<float>& pointVec,std::vector<float>& normalVec) {
    lvr2::floatArr pointArr = mesh.getVertices();
    lvr2::floatArr normalArr = mesh.getVertexNormals();

    for (int i = 0; i < mesh.numVertices() * 3; i++) {
        pointVec[i] = pointArr[i];
        normalVec[i] = normalArr[i];

    }
}

inline void OrganizedFastMeshGenerator::putStdVectorInMesh(lvr2::MeshBuffer& mesh, std::vector<float>& pointVec,std::vector<float>& normalVec) {
    lvr2::floatArr pointArr(new float(pointVec.size()));
    lvr2::floatArr normalArr(new float(normalVec.size()));
    for (int i = 0; i < pointVec.size(); i++) {
        pointArr[i] = pointVec[i];
    }

    for (int i = 0; i < normalVec.size(); i++) {
        normalArr[i] = normalVec[i];
    }

    mesh.setVertices(pointArr,pointVec.size()/3);
    mesh.setVertices(normalArr,normalVec.size()/3);


}
/*
inline void OrganizedFastMeshGenerator::putStdVectorFaceInMesh(lvr2::MeshBuffer& mesh, std::vector<int>& faceVec) {
    lvr2::indexArray face (new unsigned int(faceVec.size()));
    for (int i=0;i<faceVec.size();i++){
        face[i]=faceVec[i];
    }
    mesh.setFaceIndices(face,faceVec.size()/3);

}
*/

inline void OrganizedFastMeshGenerator::adFacetoMeshBuffer(lvr2::MeshBuffer& mesh, std::vector<int> faceVec) {
    lvr2::indexArray face(new unsigned int(mesh.numFaces()+faceVec.size()));
    face=mesh.getFaceIndices();
    for(int i =mesh.numFaces();i<mesh.numFaces()+faceVec.size(); i++){
        face[i]=faceVec[i-mesh.numFaces()];
    }
    mesh.setFaceIndices(face,mesh.numFaces()+faceVec.size());

}





void OrganizedFastMeshGenerator::fillContour(std::vector<int>& contour_indices, lvr2::MeshBuffer& mesh,std::vector<int>& fillup_indices){

    std::vector<int>::iterator c_iter;
    std::map<int, int> hole_index_map;
    std::vector<float> pointVec;
    std::vector<float> normalVec;
    lvr2MeshtoStdVector(mesh,pointVec,normalVec);

    std::cout << "Remove directly duplicate indices in contour..." << std::endl;
    // remove consecutive duplicates
    std::vector<int> clean_indices = contour_indices;
    bool clean;
    do{
        clean = true;
        std::vector<int> tmp_indices;
        int a, b;
        for(int i=0; i<clean_indices.size()-2;i++){


            a = clean_indices[i];
            b = clean_indices[i+2];
            tmp_indices.push_back(a);

            if(a == b){
                i+=2;
                clean = false;
        }
    }
    if(a != b){
      tmp_indices.push_back(*(clean_indices.end()-2));
      tmp_indices.push_back(*(clean_indices.end()-1));
    }
    clean_indices = tmp_indices;
    }
    while(!clean);

    std::cout << "Removed " << contour_indices.size() - clean_indices.size() << " duplicate indices of " << contour_indices.size() << " indices in the contour." << std::endl;

    contour_indices = clean_indices;

    for(int i=0; i<contour_indices.size();i++){
        if(contour_indices[i] == -1){
            std::cerr << "Error: Invalid point in contour at index " << i <<"!" << std::endl;
            return;
        }
    }

    // calculate the centroid of the contour
    lvr2::ColorVertex<float,int> centroid;

  for(c_iter = contour_indices.begin(); 
      c_iter != contour_indices.end(); ++ c_iter)
  {
    lvr2::ColorVertex<float,int> current_vertex;
    //what ??
    //pclToLvrVertex((*mesh_points)[*c_iter], current_vertex);
    centroid += current_vertex;
  }
  centroid /= contour_indices.size();

  lvr2::Normal<float>normal(0,0,1);

  int num_sub_contours = (int) floor(log2(contour_indices.size()));

  std::vector<std::vector<int> > hole_triangles;
  std::vector<std::vector<int> > hole_indices;

  hole_indices.push_back(contour_indices);
  int skip = 1;

    for(int i=0; i<num_sub_contours; i++) {
        std::vector<int> inner_contour;
        float scale = 1 - (float) (i + 1) / num_sub_contours;
        skip *= 2;
        for (int j = 0; j < contour_indices.size(); j += skip) {

            lvr2::ColorVertex<float, int> con_vertex(vertices[contour_indices[j]]);

            pointVec.push_back(centroid[0] + ((con_vertex[0]) - centroid[0] * scale));
            pointVec.push_back(centroid[1] + ((con_vertex[1]) - centroid[2] * scale));
            pointVec.push_back(centroid[2] + ((con_vertex[2]) - centroid[2] * scale));

            normalVec.push_back(0);
            normalVec.push_back(0);
            normalVec.push_back(1);


            int new_index = ((int) mesh_pointsBuffer.numPoints()-1);
            fillup_indices.push_back(new_index);
            inner_contour.push_back(new_index);
        }
        hole_indices.push_back(inner_contour);
    }





    std::vector<int> inner_contour;
    int new_index = ((int)pointVec.size()/3)-1;
    fillup_indices.push_back(new_index);
    inner_contour.push_back(new_index);
    hole_indices.push_back(inner_contour);

    //dirty
  for(int j=0; j< hole_indices.size()-3; j++){
    int n_j0 = hole_indices[j+0].size();
    int n_j1 = hole_indices[j+1].size();


    for(int i=0; i< n_j0 + 1; i++){
      int a = hole_indices[j  ][i % n_j0];
      int b = hole_indices[j+1][(i / 2) % n_j1];
      int c = hole_indices[j  ][(i+1) % n_j0];
      int d = hole_indices[j+1][(i+1)/2 % n_j1];

      if(i % 2 == 0){
        std::vector<int> t1(3,0); 
        t1[0] = c;
        t1[1] = b;
        t1[2] = a;
        hole_triangles.push_back(t1);
      }else{
        std::vector<int> t2(3,0);
        std::vector<int> t3(3,0);

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


  int center_index = hole_indices[hole_indices.size()-1][0];
  int j_sub = hole_indices.size()-2;
  int n_j_sub = hole_indices[j_sub].size();
  for(int i=0; i<hole_indices[j_sub].size(); i++){
    int a = hole_indices[j_sub][(i+1) % n_j_sub];
    int b = hole_indices[j_sub][i];
    int c = center_index;

    std::vector<int> t4(3,0); 
    t4[0] = a;
    t4[1] = b;
    t4[2] = c;
    hole_triangles.push_back(t4);
  }




    pcl::PointIndices::Ptr in_radius_indices (new pcl::PointIndices);

    std::vector<float> radius_distances;









    pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(mesh_pointsBuffer);
    tree->radiusSearch(mesh_pointsBuffer, 1.2, in_radius_indices->indices, radius_distances);




    std::cerr << "fitting a plane onto the hole vertices... " << std::endl;
    std::cout << "number of in-radius indices: " << in_radius_indices->indices.size() << std::endl;
    std::cout << "size of mesh-points: " << mesh_pointsBuffer.numPoints() << std::endl;

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
    std::cerr << "Size of mesh points: " << mesh_pointsBuffer.numPoints() << std::endl;

    lvr2::Normal<float> hole_normal(
            coefficients->values[0],
            coefficients->values[1],
            coefficients->values[2]);

    pcl::ProjectInliers<pcl::PointNormal> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (mesh_pointsBuffer);
    proj.setIndices(inliers);
    proj.setCopyAllData(true);
    proj.setModelCoefficients (coefficients);
    proj.filter (*mesh_pointsBuffer);

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





    //TODO Klaren wie WICHTIG BEI PROBLEMEN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    lvr2::MeshBuffer h_mesh;
    h_mesh=mesh;
    lvr2::MeshBuffer mesh_vertices;


    mesh_vertices.setVertices(mesh_pointsBuffer.getPointArray(),mesh_pointsBuffer.numPoints());
    mesh_vertices.setVertexNormals(mesh_pointsBuffer.getNormalArray());

    std::vector<int> addFace;

    for(size_t i=0; i<hole_triangles.size(); i++){
        if(hole_triangles[i].size() != 3){
            std::cerr << "wrong number of triangle indices, should be three! -- triangle index: " << i << std::endl;
            continue;
        }
        int a = hole_triangles[i][0];
        int b = hole_triangles[i][1];
        int c = hole_triangles[i][2];

        if(a < 0 || a >= mesh_vertices.size()){
            std::cerr << "invalid index:" << a << std::endl;
            continue;
        }
        if(b < 0 || b >= mesh_vertices.size()){
            std::cerr << "invalid index:" << b << std::endl;
            continue;
        }
        if(c < 0 || c >= mesh_vertices.size()){
            std::cerr << "invalid index:" << c << std::endl;
            continue;
        }
       addFace.push_back( hole_triangles[i][0]);
        addFace.push_back( hole_triangles[i][1]);
        addFace.push_back( hole_triangles[i][2]);

    }



    adFacetoMeshBuffer(mesh,addFace);



}

