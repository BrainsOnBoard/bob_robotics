# Build flags
CXXFLAGS += -std=c++14 -Wall -Wpedantic -Wextra -MMD -MP -O2 -pipe -I.

# Linking flags (-lm and -lstdc++ needed for clang)
LINK_FLAGS += -lm -lstdc++ -pthread

# Source files
SOURCES		:= imgui_widgets.cpp imgui_draw.cpp imgui.cpp misc/cpp/imgui_stdlib.cpp

ifdef WITH_LIBFREETYPE
	SOURCES += misc/freetype/imgui_freetype.cpp
endif

OBJECTS		:=$(SOURCES:.cpp=.o)
DEPS		:=$(SOURCES:.cpp=.d)
ARFLAGS		:= -rcs

.PHONY: all clean

all: libimgui.a

libimgui.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cpp %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) libimgui.a
