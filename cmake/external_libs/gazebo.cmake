BoB_add_include_directories(${GAZEBO_INCLUDE_DIRS})
BoB_add_link_libraries(${GAZEBO_LIBRARIES})
BoB_add_link_directories(${GAZEBO_LIBRARY_DIRS})
add_compile_flags(${GAZEBO_CXX_FLAGS})
