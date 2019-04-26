cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_module(BOB_MODULES common hid robots video
           EXTERNAL_LIBS opencv glfw3 glew)
