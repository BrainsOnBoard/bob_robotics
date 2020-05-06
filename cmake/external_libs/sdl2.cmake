# On Unix we use pkg-config to find SDL2, because the CMake package may not
# be present
if(UNIX)
    BoB_add_pkg_config_libraries(sdl2)
elseif(NOT SDL2_FOUND)
    message(FATAL_ERROR "Could not find SDL2")
    BoB_add_link_libraries(SDL2::SDL2)
endif()


# Sorry, Norbert ;-). I can try to help you install SFML if it helps!
#       -- Alex
BoB_deprecated(SDL2 SFML)
