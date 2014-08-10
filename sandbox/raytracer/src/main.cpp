#include <iostream>
#include "MatrixMath.hpp"
#include "CameraParameters.h"
#include "CGALRaytracer.h"
#include "Logger.h"
#include "ConfigFile.h"
#include "polymap/PolyMap.h"

int main(int argc, char** args) {

    // init and load config file
    ConfigFile* cf = new ConfigFile("../cameraparameters.cfg", true);
    
    // adjust camera parameters according to config file
	CameraParameters* cp = new CameraParameters(cf);

    // load map file
    PolyMap* map = new PolyMap("../../maps/bunny.ply");
    
    // initialize ray tracer
    CGALRaytracer* rt = new CGALRaytracer(map, cp);

    // do stuff
    
    
    // clean up
    delete rt;
    delete map;
    delete cp;
    delete cf;
    
	return 0;
}
