include ../common/flags.mk

TARGET = ant_world_test
CXXFLAGS += -L../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW

.PHONY: all clean libantworld

all: $(TARGET)

clean:
	rm -f ant_world_test

$(TARGET): $(TARGET).cc libantworld
	$(CXX) -o $@ $(TARGET).cc $(CXXFLAGS) $(LINK_FLAGS)

libantworld:
	$(MAKE) -C ../libantworld