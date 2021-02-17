find_package(GTest REQUIRED)

if(GTEST_FOUND)
    BoB_add_include_directories(${GTEST_INCLUDE_DIRS})
    BoB_add_link_libraries(${GTEST_LIBRARIES})
else()
    set(BUILD_GMOCK OFF)
    BoB_third_party(googletest)

    BoB_add_include_directories(${BOB_ROBOTICS_PATH}/third_party/googletest/googletest/include)
    BoB_add_link_libraries(gtest)
endif()
