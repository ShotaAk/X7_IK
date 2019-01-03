LIBS = -ldxl_x64_cpp
INCLUDE = -I/usr/local/include/dynamixel_sdk
CXXFLAGS= -std=c++11 


all: X7Controller.o main.o main

main.o: main.cpp
	g++ -c main.cpp $(CXXFLAGS) $(LIBS) $(INCLUDE)

X7Controller.o: X7Controller.hpp X7Controller.cpp
	g++ -c X7Controller.cpp $(CXXFLAGS) $(LIBS) $(INCLUDE)

main: main.o X7Controller.o
	g++ main.o X7Controller.o -o main $(CXXFLAGS) $(LIBS) $(INCLUDE)

clean:
	rm X7Controller.o main.o main
