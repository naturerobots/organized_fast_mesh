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
#include <pcl-1.10/pcl/surface/mls.h>

#include <pcl-1.10/pcl/surface/impl/mls.hpp>
#include <pcl-1.10/pcl/sample_consensus/method_types.h>
#include <pcl-1.10/pcl/sample_consensus/model_types.h>
#include <pcl-1.10/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.10/pcl/sample_consensus/ransac.h>
#include <pcl-1.10/pcl/sample_consensus/sac_model_plane.h>
#include <pcl-1.10/pcl/filters/project_inliers.h>
#include <pcl-1.10/pcl/surface/gp3.h>
#include <lvr2/geometry/HalfEdgeMesh.hpp>
#include <pcl-1.10/pcl/filters/extract_indices.h>
#include <pcl-1.10/pcl/surface/marching_cubes_rbf.h>

OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointNormal>& organized_scan)
: organized_scan(organized_scan)
{
  setEdgeThreshold(0.5);
}

void OrganizedFastMeshGenerator::getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh){

  // clear the vertices vector
  vertices.clear();
  mesh_points = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);


  uint32_t width = organized_scan.width;
  uint32_t height = organized_scan.height;

  index_map_index = 0;
  int index_cnt = 0;

  // clear index map
  index_map.clear();

  // add all vertices and normals to the mesh
  // also create an index map for the triangle creation
  for(uint32_t y=0; y<height; y++){
    for(uint32_t x=0; x<width; x++){

      lvr::ColorVertex<float, int> point; // point at (x,y)
      lvr::Normalf normal; // normal at (x,y);
      // get the point at (x,y)
      pcl::PointNormal p_pcl = organized_scan(x, y);

      pclToLvrVertex(p_pcl, point);
      pclToLvrNormal(p_pcl, normal);
      if(!pointExists(point) || !normalExists(normal)){
        // index maps to -1
        index_map[index_map_index] = -1;
        index_map_index++;
      }else{ // if the point exists (not nan)
        // index maps to existing vertex in the mesh
        index_map[index_map_index] = index_cnt;
        index_map_index++;
        index_cnt++;

        mesh.addVertex(point);
        mesh_points->push_back(p_pcl);
        mesh.addNormal(normal);
        vertices.push_back(point);

      }
    }
  }


  // start adding faces to the mesh
  for(uint32_t y=0; y<height; y++){  
    for(uint32_t x=0; x<width; x++){

      // get indices around the borders for a 360 degree view
      uint32_t x_right = (x == width-1) ? 0 : x+1;
      uint32_t y_bottom = (y == height-1) ? 0 : y+1;

      // get the corresponding indices in the mesh
      int idx =    index_map[toIndex(x, y)];
      int idx_r =  index_map[toIndex(x_right, y)];
      int idx_rb = index_map[toIndex(x_right, y_bottom)];
      int idx_b =  index_map[toIndex(x, y_bottom)];

      //     top              bottom
      //   triangle          triangle
      //    .___.             .   .
      //     \  |             |\
      //      \ |             | \
      //       \|             |  \
      //    .   .             .___.
      // 

      // create top triangle if all vertices exists
      if(idx != -1  && idx_rb != -1 && idx_r != -1){
        // check if there are longer edges then the threshold
        if(!hasLongEdge(idx, idx_rb, idx_r, sqr_edge_threshold))
          mesh.addTriangle(idx, idx_rb, idx_r);
      }
      // create bottom triangle if all vertices exists
      if(idx != -1 && idx_b != -1 && idx_rb != -1){
        // check if there are longer edges then the threshold
        if(!hasLongEdge(idx, idx_b, idx_rb, sqr_edge_threshold))
          mesh.addTriangle(idx, idx_b, idx_rb);
      }
    }
  }

  // finalize the mesh for the MeshBufferPointer
  mesh.finalize();
}

bool OrganizedFastMeshGenerator::getContour(std::vector<int>& contour_indices){
  // fill up robot shadow in the mesh
  bool found_contour = false;

  int height = organized_scan.height;
  int width = organized_scan.width;

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

  int height = organized_scan.height;
  int width = organized_scan.width;

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

inline bool OrganizedFastMeshGenerator::isValid(int x, int y){
  const int height = organized_scan.height;
  const int width = organized_scan.width;

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
  const int height = organized_scan.height;
  const int width = organized_scan.width;

  return
    0 <= x && x < width && 0 <= y && y < height;
}

inline bool OrganizedFastMeshGenerator::hasLongEdge(int a, int b, int c, float sqr_edge_threshold){
  lvr::ColorVertex<float, int> v_a = vertices[a];
  lvr::ColorVertex<float, int> v_b = vertices[b];
  lvr::ColorVertex<float, int> v_c = vertices[c];
  if(v_a.sqrDistance(v_b) > sqr_edge_threshold) return true;
  if(v_b.sqrDistance(v_c) > sqr_edge_threshold) return true;
  if(v_c.sqrDistance(v_a) > sqr_edge_threshold) return true;
  return false;
}

inline void OrganizedFastMeshGenerator::pclToLvrNormal(pcl::PointNormal& in, lvr::Normalf& out){
  out.x = in.normal_x;
  out.y = in.normal_y;
  out.z = in.normal_z;
}

inline void OrganizedFastMeshGenerator::lvrToPclVertex(lvr::Vertexf& vertex, lvr::Normalf& normal, pcl::PointNormal& out){
  out.x = vertex.x;
  out.y = vertex.y;
  out.z = vertex.z;
  out.normal_x = normal.x;
  out.normal_y = normal.y;
  out.normal_z = normal.z;
}

inline void OrganizedFastMeshGenerator::pclToLvrVertex(pcl::PointNormal& in, lvr::Vertexf& out){
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

inline void OrganizedFastMeshGenerator::pclToLvrVertex(pcl::PointNormal& in, lvr::ColorVertex<float, int>& out){
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

inline uint32_t OrganizedFastMeshGenerator::toIndex(int x, int y){
  uint32_t height = organized_scan.height;
  uint32_t width = organized_scan.width;

  //normalize(x, y);
  //return x*height+y;
  return y*width+x;
}

inline bool OrganizedFastMeshGenerator::pointExists(lvr::ColorVertex<float, int>& vertex){
  return
    boost::math::isfinite<float>(vertex.x) &&
    boost::math::isfinite<float>(vertex.y) &&
    boost::math::isfinite<float>(vertex.z);
}

inline bool OrganizedFastMeshGenerator::pointExists(lvr::Vertexf& vertex){
  return
    boost::math::isfinite<float>(vertex.x) &&
    boost::math::isfinite<float>(vertex.y) &&
    boost::math::isfinite<float>(vertex.z);
}

inline bool OrganizedFastMeshGenerator::normalExists(lvr::Normalf& normal){
  return
    boost::math::isfinite<float>(normal.x) &&
    boost::math::isfinite<float>(normal.y) &&
    boost::math::isfinite<float>(normal.z);
}

inline bool OrganizedFastMeshGenerator::pointExists(pcl::PointNormal& point){
  return
    boost::math::isfinite<float>(point.x) &&
    boost::math::isfinite<float>(point.y) &&
    boost::math::isfinite<float>(point.z);
}

inline void OrganizedFastMeshGenerator::normalize(int& x, int& y){
  uint32_t height = organized_scan.height;
  uint32_t width = organized_scan.width;

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



void OrganizedFastMeshGenerator::fillContour(std::vector<int>& contour_indices, lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh, std::vector<int>& fillup_indices){
  std::vector<int>::iterator c_iter;
  std::map<int, int> hole_index_map;

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
  }while(!clean);

  std::cout << "Removed " << contour_indices.size() - clean_indices.size() << " duplicate indices of " << contour_indices.size() << " indices in the contour." << std::endl;

  contour_indices = clean_indices;

  for(int i=0; i<contour_indices.size();i++){
    if(contour_indices[i] == -1){
      std::cerr << "Error: Invalid point in contour at index " << i <<"!" << std::endl;
      return;
    }
  }

  // calculate the centroid of the contour
  lvr::Vertexf centroid;

  for(c_iter = contour_indices.begin(); 
      c_iter != contour_indices.end(); ++ c_iter)
  {
    lvr::Vertexf current_vertex;
    pclToLvrVertex((*mesh_points)[*c_iter], current_vertex);
    centroid += current_vertex;
  }
  centroid /= contour_indices.size();

  pcl::PointNormal pcl_centroid;
  lvr::Normalf normal(0,0,1);
  lvrToPclVertex(centroid, normal, pcl_centroid);

  int num_sub_contours = (int) floor(log2(contour_indices.size()));

  std::vector<std::vector<int> > hole_triangles;
  std::vector<std::vector<int> > hole_indices;

  hole_indices.push_back(contour_indices);
  int skip = 1;
  for(int i=0; i<num_sub_contours; i++){
    std::vector<int> inner_contour;
    float scale = 1 - (float)(i+1)/num_sub_contours;
    skip*=2;
    for(int j=0; j<contour_indices.size(); j+=skip){

      lvr::Vertexf con_vertex(vertices[contour_indices[j]]);
      lvr::Vertexf sub_vec = centroid + ((con_vertex - centroid)*scale);
      lvr::Normalf sub_nor(0, 0, 1);
      mesh.addVertex(lvr::ColorVertex<float, int>(sub_vec));
      mesh.addNormal(sub_nor);

      pcl::PointNormal pcl_sub;
      lvrToPclVertex(sub_vec, sub_nor, pcl_sub);
      mesh_points->push_back(pcl_sub);
      int new_index = ((int)mesh_points->size())-1;
      fillup_indices.push_back(new_index);
      inner_contour.push_back(new_index);
    }
    hole_indices.push_back(inner_contour);
  }
  mesh.addVertex(lvr::ColorVertex<float, int>(centroid));
  lvr::Normalf sub_nor(0, 0, 1);
  mesh.addNormal(sub_nor);
  pcl::PointNormal pcl_sub;
  lvrToPclVertex(centroid, sub_nor, pcl_sub);
  mesh_points->push_back(pcl_sub);

  std::vector<int> inner_contour;
  int new_index = ((int)mesh_points->size())-1;
  fillup_indices.push_back(new_index);
  inner_contour.push_back(new_index);
  hole_indices.push_back(inner_contour);


  for(int j=0; j< hole_indices.size()-2; j++){
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
    mesh.addTriangle(
        hole_triangles[i][0],
        hole_triangles[i][1],
        hole_triangles[i][2]);
  }


}
