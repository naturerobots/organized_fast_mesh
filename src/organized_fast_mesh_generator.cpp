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

#include <organized_fast_mesh_generator.h>
#include <boost/math/special_functions/fpclassify.hpp>

OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointXYZ>& organized_scan)
  : organized_scan(organized_scan)
{
  setEdgeThreshold(0.5);
}

void OrganizedFastMeshGenerator::getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh){

  // clear the vertices vector
  vertices.clear();

  uint32_t width = organized_scan.width;
  uint32_t height = organized_scan.height;

  int index = 0;
  int index_cnt = 0;
  std::map<int, int> index_map;


  // add all vertices and normals to the mesh
  // also create an index map for the triangle creation
  for(uint32_t x=0; x<width; x++){
    for(uint32_t y=0; y<height; y++){
      lvr::ColorVertex<float, int> point; // point at (x,y)
      // get the point at (x,y)
      pcl::PointXYZ p_pcl = organized_scan(x, y);
      pclToLvr(p_pcl, point);
      if(!pointExists(point)){
        // index maps to -1
        index_map[index++] = -1;
      }else{ // if the point exists (not nan)
        // index maps to existing vertex in the mesh
        index_map[index++] = index_cnt++;
        mesh.addVertex(point);
        vertices.push_back(point);

        lvr::ColorVertex<float, int>
          left,         // point at (x-1, y)
          top,          // point at (x, y-1)
          right,        // point at (x+1, y)
          bottom,       // point at (x, y+1)
          bottom_right; // ponit at (x+1, y+1)
        
        // get indices around the borders for a 360 degree view
        uint32_t x_left = (x == 0) ? width-1 : x-1;
        uint32_t x_right = (x == width-1) ? 0 : x+1;
        uint32_t y_top = (y == 0) ? height-1 : y-1;
        uint32_t y_bottom = (y == height-1) ? 0 : y+1;

        // convert points to lvr
        pclToLvr(organized_scan(x_left, y), left);
        pclToLvr(organized_scan(x, y_top), top);
        pclToLvr(organized_scan(x_right, y), right);
        pclToLvr(organized_scan(x, y_bottom), bottom);
        pclToLvr(organized_scan(x_right, y_bottom), bottom_right);

        // calculate normals of the surrounding faces
        // and finally the vertex normal
        std::vector<lvr::Normal<float> > normals;
        std::vector<lvr::ColorVertex<float, int> >normal_vertices;
        normal_vertices.push_back(left);
        normal_vertices.push_back(top);
        normal_vertices.push_back(right);
        normal_vertices.push_back(bottom);
        normal_vertices.push_back(left);

        std::vector<lvr::ColorVertex<float, int> >::iterator it;
        for(it = normal_vertices.begin(); it != normal_vertices.end()-1; ++it){
          lvr::ColorVertex<float, int> a = point;
          lvr::ColorVertex<float, int> b = *it;
          lvr::ColorVertex<float, int> c = *(it+1);
          if(pointExists(a) && pointExists(b) && pointExists(c)){
            lvr::Vertex<float> ab = b-a;
            lvr::Vertex<float> ac = c-a;
            normals.push_back(lvr::Normal<float>(ab.cross(ac)));
          }
        }
        // if there are no surrounding faces and therefore no normals
        // take the null_vertex to point vector as normal
        if(normals.size() < 1){
          lvr::ColorVertex<float, int> null_vertex(0,0,0);
          mesh.addNormal(lvr::Normal<float>(point-null_vertex));
        }
        // otherwise combine all normals to one vertex normal
        else{
          lvr::Normal<float> normal(normals.front());
          std::vector<lvr::Normal<float> >::iterator n_it;
          for(n_it = normals.begin()+1; n_it!=normals.end(); n_it++){
            normal += *n_it;
          }
          normal.normalize();
          mesh.addNormal(normal);
        }
      }
    }
  }

  bool first_in_scan;
  std::vector<int> first_scan_line;

  for(uint32_t y=0; y<height; y++){
    first_in_scan = true;
    for(uint32_t x=width-1; x > 0 && first_in_scan; x--){
      if(index_map[toIndex(x, y)] != -1){
        first_in_scan = false;
        first_scan_line.push_back(toIndex(x,y));
      }
    }
  }

  lvr::ColorVertex<float, int> center;
  center.x = center.y = center.z = 0;
  for(std::vector<int>::iterator iter = first_scan_line.begin(); iter != first_scan_line.end(); ++iter)
  {
    center += vertices[index_map[*iter]];
  }
  center /= first_scan_line.size();
  mesh.addVertex(center);
  mesh.addNormal(lvr::Normal<float>(0,0,1));
  vertices.push_back(center);
  index_map[index++] = index_cnt++;
  int center_id = index - 1;
  int previous_id = first_scan_line.back();
  for(std::vector<int>::iterator iter = first_scan_line.begin(); iter != first_scan_line.end(); ++iter)
  {
    mesh.addTriangle(index_map[center_id], index_map[previous_id], index_map[*iter]);
    previous_id = *iter;
  }
  

  // start adding faces to the mesh
  for(uint32_t x=0; x<width; x++){
    for(uint32_t y=0; y<height; y++){
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

void OrganizedFastMeshGenerator::setEdgeThreshold(float dist){
  sqr_edge_threshold = dist * dist;
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

inline void OrganizedFastMeshGenerator::pclToLvr(pcl::PointXYZ& in, lvr::ColorVertex<float, int>& out){
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

inline uint32_t OrganizedFastMeshGenerator::toIndex(uint32_t x, uint32_t y){
  uint32_t height = organized_scan.height;
  return x*height+y;
}

inline bool OrganizedFastMeshGenerator::pointExists(lvr::ColorVertex<float, int>& vertex){
  return !boost::math::isnan<float>(vertex.x) && !boost::math::isnan<float>(vertex.y) && !boost::math::isnan<float>(vertex.z);
}
