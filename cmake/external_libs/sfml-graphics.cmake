# It seems like only newer versions of SFML include a CMake package,
# so use pkg-config on Unix instead, in case we don't have it
if(UNIX)
    BoB_add_pkg_config_libraries(sfml-graphics)
else()
    find_package(SFML REQUIRED graphics)
    BoB_add_link_libraries(sfml-graphics)
endif()
