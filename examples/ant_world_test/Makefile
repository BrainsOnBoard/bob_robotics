include ../../common/flags.mk
CXXFLAGS += -I../..
LINK_FLAGS += -L../../libantworld -lantworld -lglfw -lGL -lGLU -lGLEW

TARGET = ant_world_test

.PHONY: all clean libantworld

all: $(TARGET)

clean:
	rm -f ant_world_test *.d

$(TARGET): libantworld
	$(CXX) -o $@ $(TARGET).cc $(CXXFLAGS) $(LINK_FLAGS)

-include ant_world_test.d

%.d: ;

libantworld:
	$(MAKE) -C ../../libantworld