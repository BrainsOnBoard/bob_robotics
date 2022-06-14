cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(antworld)

find_package(BoBRobotics REQUIRED COMPONENTS common hid video/opengl)
BoB_module(agent.cc camera.cc render_mesh.cc render_target_input.cc
           render_target.cc renderer.cc route_ardin.cc route_continuous.cc
           snapshot_processor_ardin.cc surface.cc texture.cc world.cc)

find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SFML REQUIRED COMPONENTS graphics system window)

target_include_directories(${BOB_MODULE_TARGET} PUBLIC ${OpenCV_INCLUDE_DIRS}
                           ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS}
                           ${SFML_INCLUDE_DIR})
target_link_libraries(${BOB_MODULE_TARGET} PUBLIC ${GLEW_LIBRARIES}
                      ${OpenCV_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
                      ${SFML_LIBRARIES})
