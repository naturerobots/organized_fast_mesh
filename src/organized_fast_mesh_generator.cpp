#include <organized_fast_mesh_generator.h>

OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointXYZ>& organized_scan)
  : organized_scan(organized_scan)
{

}

void OrganizedFastMeshGenerator::getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh){

  uint32_t width = organized_scan.width;
  uint32_t height = organized_scan.height;

  ROS_INFO("start building organized mesh width: %d, height: %d", width, height);
  int index = 0;
  int index_cnt = 0;
  std::map<int, int> index_map;
  for(uint32_t x=0; x<width; x++){
    for(uint32_t y=0; y<height; y++){
      lvr::ColorVertex<float, int> point, left, top, right, bottom;
     // ROS_INFO("create vertex for (%d, %d)", x, y);
      pcl::PointXYZ p_pcl = organized_scan(x, y);

      pclToLvr(p_pcl, point);
     // ROS_INFO("to lvr");
      //ROS_INFO("Point(%d, %d): (%f, %f, %f)", x, y, p_pcl.x, p_pcl.y, p_pcl.z);
      if(!pointExists(point)){
        //ROS_INFO("Point(%d, %d): nan", x, y);
        index_map[index++] = -1;
      }else{
       // ROS_INFO("is not nan");
        index_map[index++] = index_cnt++;
        mesh.addVertex(point);
        
        // calc normal
        uint32_t x_left = (x == 0) ? width-1 : x-1;
        uint32_t x_right = (x == width-1) ? 0 : x+1;
        uint32_t y_top = (y == 0) ? height-1 : y-1;
        uint32_t y_bottom = (y == height-1) ? 0 : y+1;

        pclToLvr(organized_scan(x_left, y), left);
        pclToLvr(organized_scan(x, y_top), top);
        pclToLvr(organized_scan(x_right, y), right);
        pclToLvr(organized_scan(x, y_bottom), bottom);

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
        if(normals.size() < 1){
          lvr::ColorVertex<float, int> null_vertex(0,0,0);
          mesh.addNormal(lvr::Normal<float>(point-null_vertex));
        }else{
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

  ROS_INFO("start adding faces...");
  for(uint32_t x=0; x<width; x++){
    for(uint32_t y=0; y<height; y++){
      uint32_t x_right = (x == width-1) ? 0 : x+1;
      uint32_t y_bottom = (y == height-1) ? 0 : y+1;
      
      int idx =    index_map[toIndex(x, y)];
      int idx_r =  index_map[toIndex(x_right, y)];
      int idx_rb = index_map[toIndex(x_right, y_bottom)];
      int idx_b =  index_map[toIndex(x, y_bottom)];

      //ROS_INFO("Indixes: Index:%d, idy:%d, idx_r:%d, idx_rb:%d, idx_b:%d",i, idx, idx_r, idx_rb, idx_b);
      if(idx != -1  && idx_rb != -1 && idx_r != -1){
        mesh.addTriangle(idx, idx_rb, idx_r);
      }

      if(idx != -1 && idx_b != -1 && idx_rb != -1){
        mesh.addTriangle(idx, idx_b, idx_rb);
      }
    }
  }
  mesh.finalize();
}

void OrganizedFastMeshGenerator::pclToLvr(pcl::PointXYZ& in, lvr::ColorVertex<float, int>& out){
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

uint32_t OrganizedFastMeshGenerator::toIndex(uint32_t x, uint32_t y){
  uint32_t height = organized_scan.height;
  return x*height+y;
}

void OrganizedFastMeshGenerator::fromIndex(uint32_t index, uint32_t& x, uint32_t&y){
  uint32_t height = organized_scan.height;
  uint32_t width = organized_scan.width;
  y = (int)index%(int)width;
  x = (int)index/(int)width;
}

bool OrganizedFastMeshGenerator::pointExists(lvr::ColorVertex<float, int>& vertex){
  return !boost::math::isnan<float>(vertex.x) && !boost::math::isnan<float>(vertex.y) && !boost::math::isnan<float>(vertex.z);
}
