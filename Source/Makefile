CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall
LDFLAGS = -lGL -lglfw -lgmp -lmpfr

TARGET = path_planning
SOURCES = main.cpp geometry.cpp path_finder.cpp
HEADERS = geometry.h path_finder.h

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(TARGET) $(LDFLAGS)

clean:
	rm -f $(TARGET)

run: $(TARGET)
	./$(TARGET)
