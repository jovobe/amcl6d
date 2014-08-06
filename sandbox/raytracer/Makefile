SRC = src/
OBJ = obj/
BIN = bin/

main: logger
	g++ $(SRC)main.cpp $(OBJ)Logger.o -o $(BIN)run

logger:
	g++ $(SRC)Logger.cpp -c -o $(OBJ)Logger.o
