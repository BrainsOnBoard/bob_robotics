BoB_add_link_libraries(imgui)

# Extra libs needed
BoB_external_libraries(glew sfml-graphics)

# Suppress warning
add_compile_flags(-Wno-stringop-truncation)
