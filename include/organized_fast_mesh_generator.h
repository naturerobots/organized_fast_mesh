#ifndef ORGANIZED_FAST_MESH_GENERATOR_H_
#define ORGANIZED_FAST_MESH_GENERATOR_H_
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <reconstruction/MeshGenerator.hpp>
#include <geometry/HalfEdgeMesh.hpp>
#include <geometry/BaseMesh.hpp>
#include <geometry/ColorVertex.hpp>
#include <geometry/Normal.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/math/special_functions/fpclassify.hpp>

class OrganizedFastMeshGenerator : public lvr::MeshGenerator<lvr::ColorVertex<float, int>, lvr::Normal<float> >{

  public:
    OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointXYZ>::Ptr &organized_scan);
    virtual void getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh);
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr organized_scan;
};

#endif /* organized_fast_mesh_generator.h */
