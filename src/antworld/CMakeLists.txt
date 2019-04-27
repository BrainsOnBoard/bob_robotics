cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(SOURCES agent.cc render_mesh.cc render_target_input.cc
                   render_target.cc renderer.cc route_ardin.cc
                   route_continuous.cc snapshot_processor_ardin.cc world.cc
           BOB_MODULES common hid robots video
           EXTERNAL_LIBS opencv glfw3 glew)
