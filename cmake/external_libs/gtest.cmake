find_package(GTest REQUIRED)
BoB_add_include_directories(${GTEST_INCLUDE_DIRS})
BoB_add_link_libraries(${GTEST_LIBRARIES})
