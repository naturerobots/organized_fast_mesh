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


/**
 * \brief Generates an organized fast mesh out of an organized point cloud
 */
class OrganizedFastMeshGenerator : public lvr::MeshGenerator<lvr::ColorVertex<float, int>, lvr::Normal<float> >{

  public:
    /**
     * \brief Constrcutor
     * \param organized_scan The organized pcl point cloud 
     */
    OrganizedFastMeshGenerator(pcl::PointCloud<pcl::PointXYZ>& organized_scan);
    
    /**
     * \brief generates the organized fast mesh
     * \param a reference to the mesh to fill with the data
     */
    virtual void getMesh(lvr::BaseMesh<lvr::ColorVertex<float, int>, lvr::Normal<float> >& mesh);
    
    /**
     * \brief sets the maximum edge length to filter big leaps in the 3D depth data
     */
    void setEdgeThreshold(float dist);
  private:

    /**
     * \brief converts a pcl point into a lvr color vertex
     * \param in pcl point
     * \param out lvr point
     */
    void pclToLvr(pcl::PointXYZ& in, lvr::ColorVertex<float, int>& out);
    
    /**
     * \brief calculates the corresponding index for x and y
     * \param x x-coord
     * \param y y-coord
     * \return index for x and y
     */
    uint32_t toIndex(uint32_t x, uint32_t y);
    
    /**
     * \brief checks if the given vertex exists
     * \param vertex The vertex to check for existence
     * \return true if all coords are not nan
     */
    bool pointExists(lvr::ColorVertex<float, int>& vertex);

    /**
     * \brief tests if the face has long edges
     */
    bool hasLongEdge(int a, int b, int c, float sqr_edge_threshold);

    //! \holds the organized point cloud
    pcl::PointCloud<pcl::PointXYZ> organized_scan;
    //! \holds the vertices in the same order as in the mesh
    std::vector<lvr::ColorVertex<float, int> >vertices;
    //! \threshold value for the longe edge test
    float sqr_edge_threshold;

};

#endif /* organized_fast_mesh_generator.h */
