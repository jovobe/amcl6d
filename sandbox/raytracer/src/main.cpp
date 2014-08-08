#include <iostream>
#include "MatrixMath.hpp"
#include "CameraParameters.h"
//#include "CGALRaytracer.h"
#include "Logger.h"
#include "ConfigFile.h"

int main(int argc, char** args) {
//	CGALRaytracer rt = new CGALRaytracer();

    // init and load config file
    ConfigFile* cf = new ConfigFile("../cameraparameters.cfg", true);
    
    // adjust camera parameters according to config file
	CameraParameters* cp = new CameraParameters(cf);


    // clean up camera
    delete cp;
    // clean up config file
    delete cf;
    
    
	return 0;
}
