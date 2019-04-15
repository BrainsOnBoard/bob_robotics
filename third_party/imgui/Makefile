# Build flags
CXXFLAGS += -std=c++14 -Wall -Wpedantic -Wextra -MMD -MP -O2 -pipe -Iimgui

# Source files
SOURCES		:= imgui_widgets.cpp imgui_draw.cpp imgui.cpp misc/cpp/imgui_stdlib.cpp examples/imgui_impl_glfw.cpp examples/imgui_impl_opengl2.cpp

ifdef WITH_LIBFREETYPE
	SOURCES += misc/freetype/imgui_freetype.cpp
endif

SOURCES		:=$(addprefix imgui/, $(SOURCES))

OBJECTS		:=$(SOURCES:.cpp=.o)
DEPS		:=$(OBJECTS:.o=.d)
ARFLAGS		:= -rcs

.PHONY: all clean submodule

all: libimgui.a

submodule:
	@git submodule update --init imgui

libimgui.a: submodule $(OBJECTS)
	$(AR) $(ARFLAGS) $@ $(OBJECTS)

-include $(DEPS)

%.o: %.cpp %.d
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%.d: ;

clean:
	rm -f $(OBJECTS) $(DEPS) libimgui.a
