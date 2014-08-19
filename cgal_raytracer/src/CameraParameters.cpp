/**
 * CameraParameters.cpp
 *
 *  Created: 2010-09-29
 *  Authors: Denis Meyer, Thomas Wiemann
 *  Last modified: 2014-08-19
 *  Author: Sebastian Höffner
 */

#include "cgal_raytracer/CameraParameters.h"

CameraParameters::CameraParameters() 
{
    Logger::instance()->log("CameraParameters - constructor");

    m_minAngleH   =  80;
    m_maxAngleH   = 110;
    m_minAngleV   =  80;
    m_maxAngleV   = 110;
    m_resolutionV =  40;
    m_resolutionH =  40;
    m_focalLength =   0.4;
    m_plane_minY  = -tan(m_minAngleV * M_PI / 180) * m_focalLength;
    m_plane_maxY  =  tan(m_minAngleV * M_PI / 180) * m_focalLength;
    m_plane_minZ  = -tan(m_minAngleH * M_PI / 180) * m_focalLength;
    m_plane_maxZ  =  tan(m_maxAngleH * M_PI / 180) * m_focalLength;
    m_minRange    = 0;
    m_maxRange    = 8;
}

CameraParameters::CameraParameters(const CameraParameters &camParams) 
{
    boost::shared_lock<boost::shared_mutex> lock(m_mutex);
    Logger::instance()->log("CameraParameters - copy constructor");

    m_minAngleH   = camParams.m_minAngleH;
    m_maxAngleH   = camParams.m_maxAngleH;
    m_minAngleV   = camParams.m_minAngleV;
    m_maxAngleV   = camParams.m_maxAngleV;
    m_resolutionV = camParams.m_resolutionV;
    m_resolutionH = camParams.m_resolutionH;
    m_focalLength = camParams.m_focalLength;
    m_plane_minZ  = camParams.m_plane_minZ;
    m_plane_maxZ  = camParams.m_plane_maxZ;
    m_plane_minY  = camParams.m_plane_minY;
    m_plane_maxY  = camParams.m_plane_maxY;
    m_minRange    = camParams.m_minRange;
    m_maxRange    = camParams.m_maxRange;
}

CameraParameters::~CameraParameters() {
    Logger::instance()->log("~CameraParameters");
}

void CameraParameters::reconfigure(cgal_raytracer::CamParamConfig &config, uint32_t level)
{
    boost::unique_lock<boost::shared_mutex> lock(m_mutex);
    m_minAngleH   = config.minAngleH; 
    m_maxAngleH   = config.maxAngleH;
    m_minAngleV   = config.minAngleV;
    m_maxAngleV   = config.maxAngleV;
    m_resolutionV = config.resolutionV;
    m_resolutionH = config.resolutionH;
    m_focalLength = config.focalLength;
    m_plane_minY  = -tan(m_minAngleV * M_PI / 180) * m_focalLength;
    m_plane_maxY  =  tan(m_minAngleV * M_PI / 180) * m_focalLength;
    m_plane_minZ  = -tan(m_minAngleH * M_PI / 180) * m_focalLength;
    m_plane_maxZ  =  tan(m_maxAngleH * M_PI / 180) * m_focalLength;
    m_minRange    = config.minRange;
    m_maxRange    = config.maxRange;
}
