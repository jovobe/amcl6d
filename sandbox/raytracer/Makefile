SRC = src/
OBJ = obj/
BIN = bin/
LIB = lib/

main: raytracer logger camparam cfgfile polymap
	g++ $(SRC)main.cpp $(OBJ)Logger.o $(OBJ)ConfigFile.o $(OBJ)CameraParameters.o $(OBJ)PolyMap.o $(OBJ)CGALRaytracer.o $(OBJ)Vertex.o $(OBJ)Face.o -o $(BIN)run -lboost_system -lCGAL -frounding-math
	
logger:
	g++ $(SRC)Logger.cpp -c -o $(OBJ)Logger.o

camparam: cfgfile logger
	g++ $(SRC)CameraParameters.cpp -c -o $(OBJ)CameraParameters.o
	
cfgfile: logger
	g++ $(SRC)ConfigFile.cpp -c -o $(OBJ)ConfigFile.o
	
polymap: logger vertex face
	g++ $(SRC)polymap/PolyMap.cpp -c -o $(OBJ)PolyMap.o
	
raytracer: polymap camparam logger
	g++ $(SRC)CGALRaytracer.cpp -c -o $(OBJ)CGALRaytracer.o
	
vertex: logger
	g++ $(SRC)polymap/Vertex.cpp -c -o $(OBJ)Vertex.o
	
face: vertex logger
	g++ $(SRC)polymap/Face.cpp -c -o $(OBJ)Face.o
	
maplib: polymap
	ar rvs $(LIB)polymap.a $(OBJ)PolyMap.o $(OBJ)Face.o $(OBJ)Vertex.o $(OBJ)Logger.o

tracelib: raytracer maplib
	ar rvs $(LIB)raytracer.a $(OBJ)CameraParameters.o $(OBJ)CGALRaytracer.o $(OBJ)ConfigFile.o $(OBJ)Logger.o