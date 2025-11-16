# Makefile para Simulação de Boids
# Para usar no MSYS2 MinGW64

CXX = g++
CXXFLAGS = -Wall -O2
LIBS = -lopengl32 -lglu32 -lfreeglut
TARGET = boids
SRC = main.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SRC) $(LIBS)

clean:
	rm -f $(TARGET).exe $(TARGET)

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run