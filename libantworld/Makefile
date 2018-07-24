SOURCES		:= renderer.cc render_mesh.cc world.cc snapshot_processor_ardin.cc route_ardin.cc route_continuous.cc
OBJECTS		:=$(SOURCES:.cc=.o)
DEPS		:=$(SOURCES:.cc=.d)
CXXFLAGS	:= -std=c++14 -g -Wall -Wpedantic -Wextra -MMD -MP `pkg-config --cflags opencv`
ARFLAGS		:= -rcs

.PHONY: all clean

all: libantworld.a

libantworld.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) libantworld.a
