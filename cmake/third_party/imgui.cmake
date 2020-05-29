BoB_add_link_libraries(imgui)

# Extra libs needed
BoB_external_libraries(glew sfml-graphics)

if(GNU_TYPE_COMPILER)
    # Suppress warning
    add_compile_flags(-Wno-stringop-truncation)
endif()
