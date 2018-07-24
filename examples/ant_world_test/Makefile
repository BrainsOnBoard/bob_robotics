LIBANTWORLD_PATH := ../../libantworld

include ../../common/flags.mk
CXXFLAGS += -I../..
LINK_FLAGS += -L$(LIBANTWORLD_PATH) -lantworld -lglfw -lGL -lGLU -lGLEW

EXECUTABLE = ant_world_test

.PHONY: all clean libantworld

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d $(LIBANTWORLD_PATH)/libantworld.a
	$(CXX) -o $@ $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

$(LIBANTWORLD_PATH)/libantworld.a:
	$(MAKE) -C $(LIBANTWORLD_PATH)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d