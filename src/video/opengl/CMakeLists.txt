cmake_minimum_required(VERSION 3.10)
include(../../../cmake/BoBRobotics.cmake)
project(video/opengl)

find_package(BoBRobotics REQUIRED COMPONENTS video)
BoB_module(opengl.cc)

find_package(GLEW REQUIRED)

# Newer versions of cmake give a deprecation warning
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

target_include_directories(${BOB_MODULE_TARGET} PUBLIC
                           ${GLEW_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC
                      ${GLEW_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
