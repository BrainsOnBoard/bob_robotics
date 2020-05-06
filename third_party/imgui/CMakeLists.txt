cmake_minimum_required(VERSION 3.1)
set(SOURCES include/imgui_widgets.cpp include/imgui_draw.cpp include/imgui.cpp include/imgui_demo.cpp
            include/misc/cpp/imgui_stdlib.cpp  include/examples/imgui_impl_opengl2.cpp
            imgui_impl_sfml.cpp)
include_directories(include)

# If we have Freetype, use it
find_package(Freetype)
if(Freetype_FOUND)
    list(APPEND SOURCES include/misc/freetype/imgui_freetype.cpp)

    # Extra Freetype include path
    include_directories(${FREETYPE_INCLUDE_DIRS})

    # Export include path and library path
    set(imgui_INCLUDE_DIRS ${FREETYPE_INCLUDE_DIRS} CACHE INTERNAL "imgui: Include directories" FORCE)
    set(imgui_LIBRARIES ${FREETYPE_LIBRARIES} CACHE INTERNAL "imgui: Libraries" FORCE)
endif()

if(GNU_TYPE_COMPILER)
    # Supress warnings from inside imgui
    add_compile_flags(-Wno-class-memaccess)
endif()

# Build as static library
add_library(imgui STATIC ${SOURCES})
