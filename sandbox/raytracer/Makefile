SRC = src/
OBJ = obj/
BIN = bin/

main: logger camparam
	g++ $(SRC)main.cpp $(OBJ)Logger.o $(OBJ)ConfigFile.o -o $(BIN)run

logger:
	g++ $(SRC)Logger.cpp -c -o $(OBJ)Logger.o

camparam: cfgfile
	g++ $(SRC)CameraParameters.cpp -c -o $(OBJ)CameraParameters.o
	
cfgfile: logger
	g++ $(SRC)ConfigFile.cpp -c -o $(OBJ)ConfigFile.o
	
cfgtest: cfgfile
	g++ $(SRC)main.cpp $(OBJ)Logger.o $(OBJ)ConfigFile.o -o $(BIN)cfgtest