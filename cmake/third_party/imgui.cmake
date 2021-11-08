# Newer versions of cmake give a deprecation warning
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL)
find_package(GLEW)
find_package(SFML COMPONENTS graphics)

if(OPENGL_FOUND AND GLEW_FOUND AND SFML_FOUND)
    list(APPEND INCLUDE_DIRS ${GLEW_INCLUDE_DIRS})
    list(APPEND LIBRARIES ${GLEW_LIBRARIES} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY}
        sfml-graphics imgui)
else()
    set(FOUND FALSE)
endif()

# if(GNU_TYPE_COMPILER)
#     # Suppress warning
#     add_compile_flags(-Wno-stringop-truncation)
# endif()
