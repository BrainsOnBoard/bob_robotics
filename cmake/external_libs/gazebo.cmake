# If Gazebo is added as a dependency multiple times (e.g. from
# multiple CMakeLists.txt files) then I'm getting an error from the
# target FreeImage::FreeImage being created multiple times - AD
if(NOT TARGET FreeImage::FreeImage)
    find_package(gazebo REQUIRED)
    BoB_add_include_directories(${GAZEBO_INCLUDE_DIRS})
    BoB_add_link_libraries(${GAZEBO_LIBRARIES})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    add_compile_flags(${GAZEBO_CXX_FLAGS})
endif()
