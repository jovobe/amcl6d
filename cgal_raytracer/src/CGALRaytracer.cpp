/**
 * CGALRaytracer.cpp
 *
 *  Created: 2011-01-06
 *  Author: Thomas Wiemann
 *  Last modified: 2014-08-18
 *  Author: Sebastian Höffner
*/
 
#include "cgal_raytracer/CGALRaytracer.h"

using std::numeric_limits;

CGALRaytracer::CGALRaytracer() 
{
    m_triangleList = TriangleList();
    m_tree = NULL;
}

CGALRaytracer::~CGALRaytracer()
{
    // Free triangle list and Tree
    Logger::instance()->log("~CGALRaytracer");
    if(m_tree != NULL)
    {
        delete m_tree;
        m_tree = NULL;
    }
}

void CGALRaytracer::setMap(amcl6d_tools::Mesh* map)
{
    // lock this to avoid modification of map while raytracing
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);

    // clean up old stuff
    if(m_triangleList.size() > 0)
    {
        m_triangleList.clear();
    }

    if(m_tree != NULL)
    {
        delete m_tree;
        m_tree = NULL;
    }

    // Setup a list of triangles
    size_t triangle_count = map->mesh.faces.size();
    Logger::instance()->logX("si", "CGALRaytracer::CGALRaytracer - Faces:", 
                             triangle_count);
    K kernel;

    // count degenerated triangles
    int degen_count = 0;
    for(size_t i = 0; i < triangle_count; i++) 
    {
        // Setup CGAL triangle and store it in list
        Point a(
            map->mesh.vertices[map->mesh.faces[i].i].x, 
            map->mesh.vertices[map->mesh.faces[i].i].y, 
            map->mesh.vertices[map->mesh.faces[i].i].z  
        );
        Point b(
            map->mesh.vertices[map->mesh.faces[i].j].x, 
            map->mesh.vertices[map->mesh.faces[i].j].y, 
            map->mesh.vertices[map->mesh.faces[i].j].z  
        );
        Point c(
            map->mesh.vertices[map->mesh.faces[i].k].x, 
            map->mesh.vertices[map->mesh.faces[i].k].y, 
            map->mesh.vertices[map->mesh.faces[i].k].z  
        );
    
        Triangle tri(a, b, c);
        if(kernel.is_degenerate_3_object()(tri)) 
        {
            ++degen_count;
        } 
        else 
        {
            m_triangleList.push_back(tri);
        }
    }

    if(degen_count > 0)
    {
        Logger::instance()->logX("sis", "CGALRaytracer - Warning: found",
                                 degen_count, "degenerated triangles.");
    }
    
    // Construct AABB tree
    m_tree = new Tree(m_triangleList.begin(), m_triangleList.end());
}



void CGALRaytracer::simulatePointCloud( 
        CameraParameters* cam_param, double* matrix, 
        double** &points, int &n_points) 
{
    // lock this, so that the map does not get modified during a raytrace
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);

    // Calculate absolute pose of the camera
    double matCamPose[16];
    MMult(matrix, cam_param->m_matrixCamOrientation, matCamPose);
    Logger::instance()->log("matCamPose = ");
    Logger::instance()->log(matCamPose, 4, 4);

    // Create pure rotation matrix from given pose
    double matPoseRotation[16];
    M4copy(matrix, matPoseRotation);
    matPoseRotation[12] = matPoseRotation[13] = matPoseRotation[14] = 0.0;
    Logger::instance()->log("matPoseRotation = ");
    Logger::instance()->log(matPoseRotation, 4, 4);

    // Create pure rotation matrix from camera offset
    double matCamRotation[16];
    M4copy(cam_param->m_matrixCamOrientation, matCamRotation);
    matCamRotation[12] = matCamRotation[13] = matCamRotation[14] = 0.0;
    Logger::instance()->log("matCamRotation = ");
    Logger::instance()->log(matCamRotation, 4, 4);

    // Calculate a rotation matrix from world coordinate system
    // to the local camera coordinate system (centered internally at (0,0,0),
    // i.e. calculate the full rotation consisting of the pose orientation of
    // the robot plus the rotation of the PMD camera and invert the result.
    double matFullRotation[16];
    double matFullRotationInv[16];
    MMult(matPoseRotation, matCamRotation, matFullRotation);
    M4inv(matFullRotation, matFullRotationInv);
    Logger::instance()->log("matFullRotation = ");
    Logger::instance()->log(matFullRotation, 4, 4);

    // Calculate ray origin from cam pose
    Point ray_origin(matCamPose[12], matCamPose[13], matCamPose[14]);
    CPoint origin(ray_origin);

    // Create a set of direction vectors according to the used
    // pin hole camera model
    RayList rayList;

    double planeZ1 = cam_param->m_plane_minZ;
    double planeZ2 = cam_param->m_plane_maxZ;
    double planeY1 = cam_param->m_plane_minY;
    double planeY2 = cam_param->m_plane_maxY;

    double stepZ = (planeZ2 - planeZ1) / cam_param->m_resolutionH;
    double stepY = (planeY2 - planeY1) / cam_param->m_resolutionV;

    int i = 0;
    for(double z = planeZ2; z > planeZ1 + stepZ; z -= stepZ) 
    {
        for(double y = planeY2; y > planeY1 + stepY; y -= stepY) 
        {
            // Create image plane point and transform according to
            // current camera pose
            double imagePlanePoint[3];
            imagePlanePoint[0] = cam_param->m_focalLength;
            imagePlanePoint[1] = y;
            imagePlanePoint[2] = z;

            transformPoint(imagePlanePoint, matCamPose);

            // Create a CGAL ray
            rayList.push_back(
                    Ray(ray_origin, 
                            Point(imagePlanePoint[0],
                                  imagePlanePoint[1], 
                                  imagePlanePoint[2])));
        }
    }

    // Do raytracing: Compute closest intersection point with the
    // map for each ray
    RayIterator start = rayList.begin();
    RayIterator end = rayList.end();
    RayIterator it;

    std::vector<double*> tmp_points;

    for(it = start; it != end; it++) 
    {
        Ray query_ray = *it;

        // Get all intersections of the current ray with triangles
        // in the tree
        ObjectAndPrimitiveIDList primitives;
        try 
        {
            m_tree->all_intersections(query_ray, 
                        std::back_inserter(primitives));
        } 
        catch (...) 
        {

        }
        // Iterator through all intersections and save the closest intersection
        // point
        if(primitives.size() > 0) 
        {
            ObjectAndPrimitiveIDIterator s = primitives.begin();
            ObjectAndPrimitiveIDIterator e = primitives.end();
            ObjectAndPrimitiveIDIterator i;

            CPoint closest(1e100, 1e100, 1e100);

            for(i = s; i != e; i++) 
            {
                // Get current intersection point
                ObjectAndPrimitiveID id = *i;
                CGAL::Object obj = id.first;
                Point currentIntersection;
                if(!CGAL::assign(currentIntersection, obj)) 
                {
                    Logger::instance()->log(
                        "CGALRaytracer: Point assignment failed.");
                } 
                else 
                {
                    CPoint current(currentIntersection);
                    if(CGAL::has_smaller_distance_to_point(origin, 
                                            current, closest)) 
                    {
                        closest = current;
                    }
                }
            }

            double* p = new double[3];
            p[0] = closest.x();
            p[1] = closest.y();
            p[2] = closest.z();
            tmp_points.push_back(p);
        } 
        else 
        {
            Logger::instance()->log("CGALRaytracer - No intersections found.");
        }

    }

    // Create a double array to represent the generated point
    // cloud and copy the generated points into it
    n_points = (int) tmp_points.size();
    points = new double*[n_points];

    for(int i = 0; i < n_points; i++) 
    {
        points[i] = new double[3];
        points[i][0] = tmp_points[i][0];
        points[i][1] = tmp_points[i][1];
        points[i][2] = tmp_points[i][2];
    }
}

void CGALRaytracer::transformPoint(double point[3], double matrix[16]) 
{
    double x, y, z;
    x = point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8];
    y = point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9];
    z = point[0] * matrix[2] + point[1] * matrix[6] + point[2] * matrix[10];
    point[0] = x + matrix[12];
    point[1] = y + matrix[13];
    point[2] = z + matrix[14];
}
