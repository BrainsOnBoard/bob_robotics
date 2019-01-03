include ../make_common/bob_robotics.mk

SOURCES		:= renderer.cc render_mesh.cc render_target.cc world.cc snapshot_processor_ardin.cc route_ardin.cc route_continuous.cc
OBJECTS		:=$(SOURCES:.cc=.o)
DEPS		:=$(SOURCES:.cc=.d)
ARFLAGS		:= -rcs

# Make sure we don't have multiple definitions for BackgroundException class
CXXFLAGS += -DNO_BACKGROUND_EXCEPTION_CATCHER_DEFINITIONS

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
