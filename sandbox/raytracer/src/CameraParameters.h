/*
 * CameraParameters.h
 *
 *  Created on: 29.09.2010
 *  Author: Denis Meyer, Thomas Wiemann
 *  Modified on: 08.08.2014
 *  Author: Sebastian Höffner
 */

#ifndef CAMERAPARAMETERS_H_
#define CAMERAPARAMETERS_H_

#include <cmath>
#include "MatrixMath.hpp"
#include "Logger.h"
#include "ConfigFile.h"

class CameraParameters
{
public:
        /**
	 * Default Constructor
	 */
	CameraParameters();

	/**
	 * Constructor
	 * @param cf Configfile
	 * @param section Section
	 */
	CameraParameters(ConfigFile* cf, int section);

	/**
	 * Copy Constructor
	 * @param camParams CameraParameters
	 */
	CameraParameters(const CameraParameters &camParams);

	/**
	 * Destructor
	 */
	virtual ~CameraParameters();

	double  m_matrixCamOrientation[16];
	double 	m_minAngleH;
	double 	m_maxAngleH;
	double 	m_minAngleV;
	double 	m_maxAngleV;
	int 	m_resolutionV;
	int 	m_resolutionH;
	double 	m_focalLength;
	double 	m_plane_minZ;
	double 	m_plane_maxZ;
	double 	m_plane_minY;
	double 	m_plane_maxY;
	int 	m_deviceType;
	double 	m_maxRange;
	double 	m_minRange;

private:
	/**
	 * Parses the given Configfile
	 * @param cf Configfile
	 * @param section Section
	 */
	void parseConfigFile(ConfigFile* cf, int section);

	static const bool output;
};

#endif /* CAMERAPARAMETERS_H_ */
