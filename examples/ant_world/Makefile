LIBANTWORLD_PATH := ../../libantworld

include ../../make_common/flags.mk
CXXFLAGS += -I../..
LINK_FLAGS := -L$(LIBANTWORLD_PATH) -lantworld -lglfw -lGL -lGLU -lGLEW $(LINK_FLAGS)

EXECUTABLE := ant_world_test

.PHONY: all clean $(LIBANTWORLD_PATH)/libantworld.a

all: $(EXECUTABLE)

-include $(EXECUTABLE).d

$(EXECUTABLE): $(EXECUTABLE).cc $(EXECUTABLE).d $(LIBANTWORLD_PATH)/libantworld.a
	$(CXX) -o $@ $(EXECUTABLE).cc $(CXXFLAGS) $(LINK_FLAGS)

$(LIBANTWORLD_PATH)/libantworld.a:
	$(MAKE) -C $(LIBANTWORLD_PATH)

%.d: ;

clean:
	rm -f $(EXECUTABLE) $(EXECUTABLE).d
