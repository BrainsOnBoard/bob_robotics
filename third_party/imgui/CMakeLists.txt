cmake_minimum_required(VERSION 3.10)
project(imgui)

set(SOURCES include/imgui_widgets.cpp include/imgui_draw.cpp include/imgui.cpp include/imgui_demo.cpp
            include/misc/cpp/imgui_stdlib.cpp  include/backends/imgui_impl_opengl2.cpp
            include/imgui_tables.cpp imgui_impl_sfml.cpp)

# If we have Freetype, use it
find_package(Freetype)
if(Freetype_FOUND)
    list(APPEND SOURCES include/misc/freetype/imgui_freetype.cpp)
endif()

if(WIN32)
    # Hack needed so we can find SFML headers here
    include_directories("$ENV{VCPKG_ROOT}/installed/x64-windows/include")
else()
    # Supress warnings from inside imgui
    # add_compile_flags(-Wno-strict-overflow)
endif()

# Build as static library
add_library(imgui STATIC ${SOURCES})
target_include_directories(imgui PUBLIC include)

if(Freetype_FOUND)
    target_include_directories(imgui PUBLIC ${FREETYPE_INCLUDE_DIRS})
    target_link_libraries(imgui PUBLIC ${FREETYPE_LIBRARIES})
endif()
