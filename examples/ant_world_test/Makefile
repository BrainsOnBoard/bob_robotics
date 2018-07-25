include ../../common/flags.mk
CXXFLAGS += -I../..

LIBANTWORLD_PATH := ../../libantworld
LINK_FLAGS += -L$(LIBANTWORLD_PATH) -lantworld -lglfw -lGL -lGLU -lGLEW
LIBANTWORLD := $(LiBANTWORLD_PATH)/libantworld.a

EXECUTABLE = ant_world_test

.PHONY: all clean

all: $(EXECUTABLE)

clean:
	rm -f $(EXECUTABLE) *.d

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d $(LIBANTWORLD)
	$(CXX) -o $@ $< $(CXXFLAGS) $(LINK_FLAGS)

$.d: ;

$(LIBANTWORLD):
	$(MAKE) -C $(LIBANTWORLD_PATH)
