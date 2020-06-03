if(NOT TARGET GLEW::GLEW)
message(FATAL_ERROR "Could not find glew")
endif()

# For some reason, the CMake package appears to exist on macOS (Homebrew), but
# the headers are not correctly added. Falling back to pkg-config seems to fix
# it.
if(APPLE)
    BoB_add_pkg_config_libraries(glew)
else()
    BoB_add_link_libraries(GLEW::GLEW)
endif()
BoB_external_libraries(opengl)
