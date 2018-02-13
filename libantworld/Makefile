

ifeq ($(strip $(findstring DARWIN,$(OS_UPPER))),DARWIN)
	CXX	:=clang++
endif

SOURCES		:= renderer.cc render_mesh.cc world.cc
OBJECTS		:=$(SOURCES:.cc=.o)
DEPS		:=$(SOURCES:.cc=.d)
CXXFLAGS	:= -std=c++11 -Wall -Wpedantic -Wextra -c -MMD -MP
ARFLAGS		:= -rcs

.PHONY: clean

libantworld.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

%.o: %.cc %.d
	$(CXX) $(CXXFLAGS) -o $@ $< $(INCLUDE_FLAGS)

clean:
	rm -f $(OBJECTS) $(DEPS) libantworld.a

-include $(DEPS)