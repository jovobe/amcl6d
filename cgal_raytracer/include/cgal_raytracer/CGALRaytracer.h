/*
 * CGALRaytracer.h
 *
 * This class provides a raytracer which uses the CGAL library.
 * It needs a pose and a mesh as well as camera parameters to provide
 * a list of points achieved from the raytrace.
 *
 *  Created: 2011-01-06
 *  Author: Thomas Wiemann
 *  Last modified: 2014-08-24
 *  Author: Sebastian Höffner
 */

#ifndef CGALRAYTRACER_H_
#define CGALRAYTRACER_H_

#include <CGAL/AABB_tree.h> // must be inserted before kernel
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/Simple_cartesian.h>

#include <list>
#include <vector>
#include <limits>

#include "amcl6d_tools/Logger.h"
#include "cgal_raytracer/CameraParameters.h"

#include <eigen_conversions/eigen_msg.h>  

#include "geometry_msgs/Pose.h"
#include "amcl6d_tools/Mesh.h"

#include <boost/thread/shared_mutex.hpp>

typedef CGAL::Simple_cartesian<double> K;
typedef CGAL::Point_3<K> CPoint;

typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;

typedef std::list<Triangle> TriangleList;
typedef std::list<Triangle>::iterator TriangleIterator;
typedef std::list<Ray> RayList;
typedef std::list<Ray>::iterator RayIterator;

typedef CGAL::AABB_triangle_primitive<K, TriangleIterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Object_and_primitive_id ObjectAndPrimitiveID;
typedef Tree::Primitive_id PrimitiveID;

typedef std::list<PrimitiveID> PrimitiveIDList;
typedef std::list<ObjectAndPrimitiveID> ObjectAndPrimitiveIDList;
typedef std::list<PrimitiveID>::iterator PrimitiveIDIterator;
typedef std::list<ObjectAndPrimitiveID>::iterator ObjectAndPrimitiveIDIterator;

class CGALRaytracer {
public:
    /**
     * Default constructor. Initializes the Trianglelist.
     */
    CGALRaytracer();

    /**
     * Destructor.
     */
    virtual ~CGALRaytracer();
    
    /**
     * Updates the map. This method acquires a boost::unique_lock so that
     * no raytraces are in progress while the map gets changed.
     *
     * @param map the new map mesh.
     */
    void setMap(amcl6d_tools::Mesh* map);
    
    /**
     * Simulates a point cloud, i.e. does the ray trace for the given
     * camera parameters.
     */
    void simulatePointCloud(CameraParameters* cam_params, 
                            geometry_msgs::Pose pose,/* double* matrix, */
                            double** &points, int &n_points);

private:
    /**
     * Transforms the point with the given matrix.
     */
    void transformPoint(double point[3], double matrix[16]);

    Tree* m_tree;
    TriangleList m_triangleList;

    boost::shared_mutex m_mutex;

};

#endif /* CGALRAYTRACER_H_ */
