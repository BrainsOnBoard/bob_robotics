BoB_add_link_libraries(imgui)

# Extra libs needed
BoB_external_libraries(glew sfml-graphics)

# Suppress warning
if(GNU_TYPE_COMPILER)
    add_compile_flags(-Wno-stringop-truncation)
endif()
