if(NOT TARGET GLEW::GLEW)
message(FATAL_ERROR "Could not find glew")
endif()

BoB_add_link_libraries(GLEW::GLEW)
BoB_external_libraries(opengl)
