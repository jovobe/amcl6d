/*
 * CGALRaytracer.h
 *
 *  Created on: 06.01.2011
 *      Author: Thomas Wiemann
 *  Modified:   10.08.2014
 *      Author: Sebastian Höffner
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

#include "MatrixMath.hpp"
#include "Logger.h"

#include "CameraParameters.h"
#include "polymap/PolyMap.h"
#include "polymap/Face.h"
#include "polymap/Vertex.h"


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
	CGALRaytracer(PolyMap *map, CameraParameters* camParams);
	void simulatePointCloud(double* matrix, double** &points, int &n_points);

	virtual ~CGALRaytracer();

private:
	void transformPoint(double point[3], double matrix[16]);

	CameraParameters* m_camParams;
	Tree* m_tree;
	TriangleList m_triangleList;

};

#endif /* CGALRAYTRACER_H_ */
