cmake_minimum_required(VERSION 3.1)
set(SOURCES include/imgui_widgets.cpp include/imgui_draw.cpp include/imgui.cpp
            include/misc/cpp/imgui_stdlib.cpp include/examples/imgui_impl_glfw.cpp
            include/examples/imgui_impl_opengl2.cpp)
include_directories(include)
if(WITH_LIBFREETYPE)
    list(APPEND SOURCES include/misc/freetype/imgui_freetype.cpp)
endif()

add_library(imgui STATIC ${SOURCES})
