TARGET = ant_world_test
CXXFLAGS := -std=c++14 -g -Wall -Wpedantic -L../libantworld -lantworld -lglfw \
		-lGL -lGLU -lGLEW -lstdc++ -lm -pthread `pkg-config --cflags --libs opencv`

.PHONY: all clean libantworld

all: $(TARGET)

clean:
	rm -f ant_world_test

$(TARGET): $(TARGET).cc libantworld
	$(CXX) -o $@ $(TARGET).cc $(CXXFLAGS)

libantworld:
	$(MAKE) -C ../libantworld