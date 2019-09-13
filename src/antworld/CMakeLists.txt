cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES agent.cc camera.cc render_mesh.cc render_target_input.cc
                   render_target.cc render_target_hex_display.cc renderer.cc renderer_stereo.cc route_ardin.cc
                   route_continuous.cc snapshot_processor_ardin.cc 
                   surface.cc texture.cc world.cc
           BOB_MODULES common hid robots video/opengl
           EXTERNAL_LIBS opencv glew sfml-graphics)
