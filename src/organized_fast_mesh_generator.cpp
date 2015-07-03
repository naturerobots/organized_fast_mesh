#include <organized_fast_mesh_generator.h>

OrganizedFastMeshGenerator::OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointXYZ>::Ptr& organized_scan)
  : organized_scan(organized_scan)
{

}

void OrganizedFastMeshGenerator::getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh){
  lvr::MeshBufferPtr mesh_buffer_ptr(new lvr::MeshBuffer);
  lvr::HalfEdgeMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> > hem(mesh_buffer_ptr);

  uint32_t width = organized_scan->width;
  uint32_t height = organized_scan->height;

  int index = 0;
  int index_cnt = 0;
  std::vector<int> index_map;
  for(uint32_t y=0; y<width; y++){
    for(uint32_t x=0; x<height; x++){
      pcl::PointXYZ p = (*organized_scan)(x, y);
      if(boost::math::isnan<float>(p.x)){
        index_map[index++] = -1;
      }else{
        index_map[index++] = index_cnt++;
        hem.addVertex(lvr::ColorVertex<float, int>(p.x, p.y, p.z));
      //TODO check if index out of bounds    
        lvr::ColorVertex<float, int> left = (*organized_scan)(x-1, y);
        lvr::ColorVertex<float, int> top = (*organized_scan)(x, y-1);
        lvr::ColorVertex<float, int> right = (*organized_scan)(x+1, y);
        lvr::ColorVertex<float, int> bottom = (*organized_scan)(x, y+1);
        
        std::vector<lvr::Normal<float> > normals;
        std::vector<lvr::ColorVertex<float, int> >normal_vertices;
        normal_vertices.push_back(left);
        normal_vertoces.push_back(top);
        normal_vertices.push_back(right);
        normal_vertices.push_back(bottom);
        normal_vertices.push_back(left);

        std::vector<lvr::ColorVertex<float, int> >::iterator it;
        for(it = normal_vertices.begin(); it != normal_vertices.end()-1; ++it){
          lvr::ColorVertex a = p;
          lvr::ColorVertex b = *it;
          lvr::ColorVertex c = *(it+1);
          if(pointExists(a) && pointExists(b) && pointExists(c)){
            lvr::Vertex<float> ab = b-a;
            lvr::Vertex<float> ac = c-a;
            lvr::Vertex<float> n = ab.cross(ac);
            lvr::Normal<float> n0(n.x, n.y, n.z);
            normals.push_back(n0);
          }
        }
      }
    }
  }

}

bool OrganziedFastMeshGenerator::pointExists(lvr::ColorVertex<float>& vertex){
  return !boost::math::isnan<float>(vertex.x) && !boost::math::isnan<float>(vertex.y) && !boost::math::isnan<float>(vertex.z);
}
