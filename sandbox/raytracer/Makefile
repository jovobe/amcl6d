SRC = src/
OBJ = obj/
BIN = bin/

main: logger camparam
	g++ $(SRC)main.cpp $(OBJ)Logger.o $(OBJ)ConfigFile.o $(OBJ)CameraParameters.o -o $(BIN)run

logger:
	g++ $(SRC)Logger.cpp -c -o $(OBJ)Logger.o

camparam: cfgfile logger
	g++ $(SRC)CameraParameters.cpp -c -o $(OBJ)CameraParameters.o
	
cfgfile: logger
	g++ $(SRC)ConfigFile.cpp -c -o $(OBJ)ConfigFile.o
	