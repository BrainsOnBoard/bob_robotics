

ifeq ($(strip $(findstring DARWIN,$(OS_UPPER))),DARWIN)
	CXX	:=clang++
endif

SOURCES		:= renderer.cc render_mesh.cc world.cc snapshot_processor_ardin.cc snapshot_processor_hog.cc route_ardin.cc route_continuous.cc
OBJECTS		:=$(SOURCES:.cc=.o)
DEPS		:=$(SOURCES:.cc=.d)
CXXFLAGS	:= -std=c++14 -Wall -Wpedantic -Wextra -MMD -MP `pkg-config --cflags opencv`
ARFLAGS		:= -rcs

.PHONY: clean

libantworld.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) libantworld.a