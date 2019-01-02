INCLUDE = -I/usr/local/include/dynamixel_sdk
LIBRARIES  += -ldxl_x64_cpp
CXXFLAGS= -std=c++11 

X7Controller.o: X7Controller.hpp X7Controller.cpp
	g++ -c X7Controller.cpp $(INCLUDE) $(LIBRARIES) 

main: main.cpp X7Controller.o
	g++ main.cpp X7Controller.o -o main
