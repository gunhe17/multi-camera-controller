CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -Wextra -pthread

SRC = main.cpp
INCLUDES = -I./include -I./capture -I./util

OUT = multicam

all:
	$(CXX) $(CXXFLAGS) $(SRC) $(INCLUDES) -o $(OUT)

clean:
	rm -f $(OUT)
