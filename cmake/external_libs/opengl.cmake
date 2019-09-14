# Newer versions of cmake give a deprecation warning
set(OpenGL_GL_PREFERENCE LEGACY)

find_package(OpenGL REQUIRED)
BoB_add_include_directories(${OPENGL_INCLUDE_DIR})
BoB_add_link_libraries(${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
