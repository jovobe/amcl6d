#include <iostream>
#include "MatrixMath.hpp"
//#include "CameraParameters.h"
//#include "CGALRaytracer.h"
#include "Logger.h"
#include "ConfigFile.h"

int main(int argc, char** args) {
//	CameraParameters cp = new CameraParameters();
//	CGALRaytracer rt = new CGALRaytracer();
	Logger::instance()->log("Testlog.");
    ConfigFile* cf = new ConfigFile("cameraparameters.cfg", true);
    delete cf;
	return 0;
}
