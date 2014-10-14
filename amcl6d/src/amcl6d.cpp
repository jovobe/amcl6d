#include "amcl6d/amcl6d.h"

amcl6d::amcl6d(ros::NodeHandle nodehandle, ros::CallbackQueue* queue)
{
    Logger::instance()->log("[AMCL] Initializing...");
    
    m_node_handle           = nodehandle;
    m_queue                 = queue;
    m_sample_number         = 100;
    m_pose_publisher        = m_node_handle.advertise<geometry_msgs::PoseArray>("pose_samples", 1000);
    m_best_pose_publisher   = m_node_handle.advertise<geometry_msgs::PoseStamped>("pose_hypothesis", 1000);
    m_poses.header.frame_id = m_current_best_pose.header.frame_id = "world";
    m_moved                 = false;
    m_has_guess             = false;
    m_factory               = new pose_factory();
    m_distribution          = std::normal_distribution<double>(0.0, 1.0);
    m_service_client        = m_node_handle.serviceClient<cgal_raytracer::RaytraceAtPose>("raytrace_at_pose");

    // TODO tweak params
    m_discard_percentage        = 0.4;
    m_close_respawn_percentage  = 0.25;
    m_top_percentage            = 0.1;
    m_low_threshold             = 0.0005;

    m_iterations = 0;

    Logger::instance()->log("[AMCL] Initialized:");
    Logger::instance()->logX("si", "[AMCL]   Sample number:  ", m_sample_number);
    Logger::instance()->logX("ss", "[AMCL]   Frame:          ", m_poses.header.frame_id.c_str());
    Logger::instance()->logX("ss", "[AMCL]   PoseArray topic:", "pose_samples");
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
    Logger::instance()->log("[AMCL] Shut down.");
}

void amcl6d::clear()
{
    m_poses.poses.clear();
}

void amcl6d::publish()
{
    m_pose_publisher.publish(m_poses);
    if(m_has_guess)
    {
        m_best_pose_publisher.publish(m_current_best_pose);
    }
}

void amcl6d::spinOnce()
{
    m_queue->callOne();
}

#include <iostream>
void amcl6d::update_poses()
{
    // get current raytrace // TODO get from robot
    m_current_pose.set_raytrace(issue_raytrace(m_current_pose.get_pose()));
    m_current_pose.normalize_raytrace();
    if(!prepare_kd_tree())
    {
        return;
    }

    // update samples
    #pragma omp parallel for
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
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

    // evaluate raytraces (i.e. get new likelihood)
    ros::Time now = ros::Time::now();
    Logger::instance()->log("[AMCL] Starting raytraces.");
    #pragma omp parallel for
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        sensor_msgs::PointCloud pcl_result = issue_raytrace(m_pose_samples[i].get_pose());
        if(pcl_result.points.size() > 0)
        {
            m_pose_samples[i].set_raytrace(pcl_result);
            m_pose_samples[i].normalize_raytrace();
            double likelihood = 1 / evaluate_sample(m_pose_samples[i]);
            m_pose_samples[i].set_likelihood(likelihood);
        }
    }
    Logger::instance()->logX("sds", "[AMCL] Raytraces done. Time elapsed:", ros::Duration(ros::Time::now() - now).toSec(), "s.");
    
    // find normalization factor (marginalize over likelihoods)
    double norm = 0;
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        norm += m_pose_samples[i].get_likelihood() * m_pose_samples[i].get_probability();
    }

    // calculate new probablities: likelihood * prior / normalization
    #pragma omp parallel for
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        m_pose_samples[i].set_probability(m_pose_samples[i].get_likelihood() * m_pose_samples[i].get_probability() / norm);
    }
    
    /**
    std::cout << "Posterior: " << std::endl;
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
       std::cout << m_pose_samples[i].get_probability() << ", ";
    }
    std::cout << std::endl;
    /**/

    // sort particles by posterior probability
    std::sort(m_pose_samples.begin(), m_pose_samples.end(), [](pose_sample left, pose_sample right){ return left.get_probability() > right.get_probability(); });
    
    double sum = 0;
    for(int i = 0; i < m_pose_samples.size(); ++i)
    {
        sum += m_pose_samples[i].get_probability();
        //std::cout << m_pose_samples[i].get_probability() << ", " << std::endl;
    }

    std::cout << "Posterior sum: " << sum << std::endl;

    m_current_best_pose.pose = m_pose_samples[0].get_pose();

    // RESPAWN
    int top_values_last = m_pose_samples.size() * m_top_percentage;

    std::cout << std::endl << "Stats:" << std::endl
                           << "  Keeping 0 to " << m_pose_samples.size() * m_top_percentage << std::endl
                           << "  Respawning close from " << m_pose_samples.size() * (1-m_close_respawn_percentage) << std::endl
                           << "  Respawning random from " << m_pose_samples.size() * (1-m_discard_percentage) << std::endl;

    for(int i = m_pose_samples.size()-1; i > m_pose_samples.size() * m_top_percentage; i--)
    {
        // respawn close to current best
        if(i > m_pose_samples.size() * (1 - m_close_respawn_percentage))
        {
            std::cout << i << " c, ";
            int idx = (int)((double)rand() / RAND_MAX * top_values_last);
            geometry_msgs::Pose n_pose = m_factory->generate_pose_near(m_pose_samples[idx].get_pose());
            m_pose_samples[i].set_pose(n_pose);
            m_pose_samples[i].set_probability(1.0 / m_sample_number);
            m_poses.poses[i] = m_pose_samples[i].get_pose();
        }
        // respawn random samples and improbable poses
        else if(i > m_pose_samples.size() * m_discard_percentage || m_pose_samples[i].get_probability() < m_low_threshold)
        {
            std::cout << i << " r, ";
            m_pose_samples[i].set_pose(m_factory->generate_random_pose());
            m_pose_samples[i].set_probability(1.0 / m_sample_number);
            m_poses.poses[i] = m_pose_samples[i].get_pose();
        }
        // else keep sample
        else
        {
        }
    }

    m_iterations++;
    std::cout << "Iterations: " << m_iterations << std::endl;
    std::cout << "Best guess ("<< m_pose_samples[0].get_probability() << "): "  << m_pose_samples[0].get_pose() << std::endl;

    m_has_guess = true;
    m_moved = true;
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

double amcl6d::evaluate_sample(pose_sample sample)
{
    int k = 1;
    double result = 0;

    std::vector<float> dst(k);
    dst.reserve(k);
    std::vector<int> ind(k);
    ind.reserve(k);
    sensor_msgs::PointCloud pcl = sample.get_raytrace();
    if(pcl.points.size() == 0)
    {
        return 1e10;
    }

    for(int i = 0; i < pcl.points.size(); ++i)
    {
        dst.erase(dst.begin(), dst.end());
        ind.erase(ind.begin(), ind.end());
        pcl::PointXYZ pt;
        pt.x = pcl.points[i].x;
        pt.y = pcl.points[i].y;
        pt.z = pcl.points[i].z;
        int neighbours = m_kd_tree.nearestKSearch(pt, k, ind, dst);
        double avg = 0;
        for(int j = 0; j < neighbours; ++j)
        {
            avg += dst[j]/neighbours;
        }
        result += avg;
    }
    result /= pcl.points.size();

    return result;
}

bool amcl6d::prepare_kd_tree()
{
    // this is needed since the rest uses the old and simple pointcloud
    // could be changed by refactoring the whole node as well
    // as the raytracer service

    // set up different clouds
    sensor_msgs::PointCloud points;
    sensor_msgs::PointCloud2 points2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // fill first cloud with the data
    points.points = m_current_pose.get_raytrace().points;
    if(points.points.size() == 0) 
    {
        Logger::instance()->log("[AMCL] No sensor input available. Move along. (Aborting.)");
        return false;
    }

    // convert through
    sensor_msgs::convertPointCloudToPointCloud2(points, points2);
    pcl::fromROSMsg(points2, *(ptr.get()));

    // set up kd_tree
    m_kd_tree = pcl::KdTreeFLANN<pcl::PointXYZ>(false);
    m_kd_tree.setInputCloud(ptr);

    return true;
}

void amcl6d::generate_poses()
{
    Logger::instance()->log("[AMCL] Generating poses...");
    if(m_factory == NULL)
    {
        Logger::instance()->log("[AMCL] Can't generate poses: No factory.");
        return;
    }

    clear();

    m_pose_samples.resize(m_sample_number);
    m_poses.poses.resize(m_sample_number);
    #pragma omp parallel for
    for(int i = 0; i < m_sample_number; ++i)
    {
        m_pose_samples[i].set_pose(m_factory->generate_random_pose());
        m_pose_samples[i].set_probability(1.0 / m_sample_number);
        m_poses.poses[i] = m_pose_samples[i].get_pose();
    }

    init_mu();
    init_covariance();
    Logger::instance()->logX("sis","[AMCL]", m_sample_number, "poses generated.");
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
    
    Eigen::Vector6d result = m_cholesky_decomp.transpose() * values + m_mu;

    return result;
}

void amcl6d::update_cholesky_decomposition()
{
    m_cholesky_decomp = m_covar.llt().matrixL();
    Logger::instance()->log("Cholesky decomposition:");
    Logger::instance()->log(m_cholesky_decomp.data(), 6, 6);
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
    Logger::instance()->log("[AMCL] Covariance matrix set:");
    Logger::instance()->log(m_covar.data(), 6, 6);
    update_cholesky_decomposition();
    Logger::instance()->log("[AMCL] Cholesky decomposition calculated.");
}

// TODO get from robot
void amcl6d::init_mu()
{
    m_mu << 0, 0, 0, 0, 0, 0;
    Logger::instance()->log("[AMCL] mu set.");
}

void amcl6d::mesh_callback(const amcl6d_tools::Mesh::ConstPtr& message)
{
    if(m_factory == NULL || message->mesh.vertices.size() == m_mesh.mesh.vertices.size())
    {
        return;
    }
    set_mesh(amcl6d_tools::Mesh(*message.get()));
}

void amcl6d::set_mesh(amcl6d_tools::Mesh mesh)
{
    Logger::instance()->log("[AMCL] Received new mesh.");

    m_mesh = mesh;
    m_factory->set_bounds(m_mesh);

    m_has_mesh = true;
    m_has_guess = false;
    m_moved = false;
    generate_poses();
}

bool amcl6d::has_mesh()
{
    return m_has_mesh;
}

void amcl6d::move_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    m_last_pose = geometry_msgs::Pose(m_current_pose.get_pose());
    geometry_msgs::Pose current_pose = geometry_msgs::Pose((*pose_msg.get()).pose);

    // calculate difference between last and current pose
    Eigen::Vector3d    last_position(m_last_pose.position.x, 
                                     m_last_pose.position.y, 
                                     m_last_pose.position.z);
    Eigen::Quaterniond last_orientation(m_last_pose.orientation.w, 
                                        m_last_pose.orientation.x, 
                                        m_last_pose.orientation.y, 
                                        m_last_pose.orientation.z);
    Eigen::Vector3d    current_position(current_pose.position.x, 
                                        current_pose.position.y, 
                                        current_pose.position.z);
    Eigen::Quaterniond current_orientation(current_pose.orientation.w, 
                                           current_pose.orientation.x, 
                                           current_pose.orientation.y, 
                                           current_pose.orientation.z);
    m_diff_position    = current_position - last_position;
    m_diff_orientation = last_orientation * current_orientation.inverse();

    m_current_pose.set_pose(current_pose);

    // if there is a difference, provide an update
    double diff = m_diff_orientation.vec().squaredNorm() + m_diff_position.squaredNorm();

    if(diff > 0 && m_moved)
    {
        Logger::instance()->log("[AMCL] Move registered.");
        update_poses();
        return;
    }

    m_moved = true;
}

geometry_msgs::PoseStamped amcl6d::get_best()
{
    return m_current_best_pose;
}
