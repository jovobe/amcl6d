#include "amcl6d/amcl6d.h"

void amcl6d::pose_sample::set_pose(geometry_msgs::Pose pose) 
{
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);  
    this->pose = pose;
}

geometry_msgs::Pose amcl6d::pose_sample::get_pose()
{
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return pose;
}

void amcl6d::pose_sample::set_probability(double probability)
{
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    this->probability = probability;
}

double amcl6d::pose_sample::get_probability()
{
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    return probability;
}

void amcl6d::pose_sample::update_pose(double add_x, double add_y, double add_z, Eigen::Quaterniond set_orientation)
{
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    pose.position.x += add_x;
    pose.position.y += add_y;
    pose.position.z += add_z;
    pose.orientation.x = set_orientation.x();
    pose.orientation.y = set_orientation.y();
    pose.orientation.z = set_orientation.z();
    pose.orientation.w = set_orientation.w();
}

amcl6d::pose_sample::pose_sample()
{

}

amcl6d::pose_sample::pose_sample(const pose_sample& copy)
{
    set_pose(copy.pose);
    set_probability(copy.probability);
}

amcl6d::amcl6d(ros::NodeHandle nodehandle)
{
    m_node_handle = nodehandle;
    m_sample_number = 100;
    m_pose_publisher = m_node_handle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    m_poses.header.frame_id = "world";
    m_factory = new pose_factory();
    m_distribution = std::normal_distribution<double>(0.0, 1.0);

    m_service_client = m_node_handle.serviceClient<cgal_raytracer::RaytraceAtPose>("raytrace_at_pose");
}

amcl6d::~amcl6d() {
    // clear published poses and publish cleared poses
    clear();
    publish();

    if(m_factory != NULL)
    {
        delete m_factory;
        m_factory = NULL;
    }
}

void amcl6d::clear()
{
    m_poses.poses.clear();
}

void amcl6d::publish()
{
    m_pose_publisher.publish(m_poses);
}

void amcl6d::update_poses()
{
    // get current raytrace
    m_current_raytrace = issue_raytrace(m_current_pose);

    // update samples
    #pragma omp parallel for
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        // TODO this might be wrong
        m_pose_samples[i].set_probability(m_pose_samples[i].get_probability() / (double)m_sample_number);

        // sample noise according to covariances and update poses
        Eigen::Vector6d noise = sample();
        
        geometry_msgs::Pose sample_pose = m_pose_samples[i].get_pose();
        Eigen::Quaterniond orientation(sample_pose.orientation.w,
                                       sample_pose.orientation.x,
                                       sample_pose.orientation.y,
                                       sample_pose.orientation.z);
        
        Eigen::Quaterniond n_orientation = m_diff_orientation * orientation;
        
        Eigen::Quaterniond rotZ(Eigen::AngleAxisd(noise(3), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond rotY(Eigen::AngleAxisd(noise(4), Eigen::Vector3d::UnitY()));
        Eigen::Quaterniond rotX(Eigen::AngleAxisd(noise(5), Eigen::Vector3d::UnitX()));

        n_orientation = rotX * rotY * rotZ * n_orientation;
        
        m_pose_samples[i].update_pose(
                    m_diff_position.x() + noise(0),
                    m_diff_position.y() + noise(1),
                    m_diff_position.z() + noise(2),
                    n_orientation
                );
                
        m_poses.poses[i] = m_pose_samples[i].get_pose();
    }

    // TODO change ROS_INFO to ROS_DEBUG
    clock_t start = clock();
    ROS_INFO("Starting raytraces.");
    #pragma omp parallel for
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        sensor_msgs::PointCloud pcl_result = issue_raytrace(m_pose_samples[i].get_pose());
        if(pcl_result.points.size() > 0)
        {
            double probability = evaluate_raytrace(pcl_result);
//            ROS_INFO("Value: %f", probability);
            m_pose_samples[i].set_probability(probability);
        }
    }
    ROS_INFO("Raytraces done.");
    clock_t stop = clock();
    ROS_INFO("Time elapsed: %f s", double(stop - start) / CLOCKS_PER_SEC);
    
    // TODO particle regeneration
}

sensor_msgs::PointCloud amcl6d::issue_raytrace(geometry_msgs::Pose pose)
{
    cgal_raytracer::RaytraceAtPose srv;
    srv.request.pose = pose;
    sensor_msgs::PointCloud pcl_result;

    if(m_service_client.call(srv))
    {
        pcl_result = srv.response.raytrace;
    }

    return pcl_result;
}

double amcl6d::evaluate_raytrace(sensor_msgs::PointCloud result)
{
    double diff_squared = 0;
    int num = std::min(m_current_raytrace.points.size(), result.points.size());
    for(int i = 0; i < num; i++)
    {
        diff_squared += (m_current_raytrace.points[i].x - result.points[i].x)
                      * (m_current_raytrace.points[i].x - result.points[i].x)
                      + (m_current_raytrace.points[i].y - result.points[i].y)
                      * (m_current_raytrace.points[i].y - result.points[i].y)
                      + (m_current_raytrace.points[i].z - result.points[i].z)
                      * (m_current_raytrace.points[i].z - result.points[i].z);
    }
   // ROS_INFO("diff_squared: %f, num: %d", diff_squared, num);
    
    return num? diff_squared / (double) num : 0;
}

void amcl6d::generate_poses()
{
    if(m_factory == NULL)
    {
        ROS_INFO("amcl6d::generate_poses(): no factory.");
        return;
    }

    clear();

    m_pose_samples.resize(m_sample_number);
    m_poses.poses.resize(m_sample_number);
    #pragma mop parallel for
    for(int i = 0; i < m_sample_number; ++i)
    {
        m_pose_samples[i].set_pose(m_factory->generate_random_pose());
        m_pose_samples[i].set_probability(1.0 / m_sample_number);
        m_poses.poses[i] = m_pose_samples[i].get_pose();
    }
    init_mu();
    init_covariance();
}

Eigen::Vector6d amcl6d::sample()
{
    Eigen::Vector6d values;
    values << m_distribution(m_generator), 
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator),
              m_distribution(m_generator);
    
    Eigen::Vector6d result = (values.transpose() * m_cholesky_decomp) + m_mu.transpose();

    /* 
    std::cout << "rand:" << std::endl << values << std::endl; 
    std::cout << "transpose" << std::endl << values.transpose()<<std::endl;
    std::cout << "mu:" << std::endl << m_mu << std::endl; 
    std::cout << "transpose" << std::endl << m_mu.transpose()<<std::endl;
    std::cout << "decomp" << std::endl << m_cholesky_decomp << std::endl;
    std::cout << "values * decomp" << std::endl << values.transpose() * m_cholesky_decomp << std::endl;
    std::cout << "result:" << std::endl;
    std::cout << result << std::endl;*/

    return result;
}

void amcl6d::update_cholesky_decomposition()
{
    m_cholesky_decomp = m_covar.llt().matrixL();
}

// TODO get from robot
void amcl6d::init_covariance()
{
    m_covar << 0.01, 0, 0, 0, 0, 0,
               0, 0.01, 0, 0, 0, 0,
               0, 0, 0.01, 0, 0, 0,
               0, 0, 0, 0.01, 0, 0,
               0, 0, 0, 0, 0.01, 0,
               0, 0, 0, 0, 0, 0.01;
    update_cholesky_decomposition();
    ROS_INFO("Updated covariances and cholesky decomposition.");
}

// TODO get from robot
void amcl6d::init_mu()
{
    m_mu << 0, 0, 0, 0, 0, 0;
    ROS_INFO("Updated mu's.");
}

void amcl6d::mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
{
    if(m_factory == NULL || message->mesh.vertices.size() == m_mesh.mesh.vertices.size())
    {
        return;
    }
    m_mesh = amcl6d_tools::Mesh(*message.get());
    m_factory->set_bounds(m_mesh);
    // TODO pose reset: avoid update when first pose arrives later!
    
    generate_poses();
}

void amcl6d::move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    m_last_pose = geometry_msgs::Pose(m_current_pose);
    m_current_pose = geometry_msgs::Pose((*pose_msg.get()).pose);

    // calculate difference between last and current pose
    Eigen::Vector3d    last_position(m_last_pose.position.x, 
                                     m_last_pose.position.y, 
                                     m_last_pose.position.z);
    Eigen::Quaterniond last_orientation(m_last_pose.orientation.w, 
                                        m_last_pose.orientation.x, 
                                        m_last_pose.orientation.y, 
                                        m_last_pose.orientation.z);
    Eigen::Vector3d    current_position(m_current_pose.position.x, 
                                        m_current_pose.position.y, 
                                        m_current_pose.position.z);
    Eigen::Quaterniond current_orientation(m_current_pose.orientation.w, 
                                           m_current_pose.orientation.x, 
                                           m_current_pose.orientation.y, 
                                           m_current_pose.orientation.z);
    m_diff_position    = current_position - last_position;
    m_diff_orientation = last_orientation * current_orientation.inverse();

    // if there is a difference, provide an update
    double diff = m_diff_orientation.vec().squaredNorm() + m_diff_position.squaredNorm();

    if(diff > 0)
    {
        ROS_DEBUG("Moved.");
        update_poses();
    }
}

