cmake_minimum_required(VERSION 3.10)
include(../cmake/BoBRobotics.cmake)
project(tests)

find_package(BoBRobotics QUIET REQUIRED COMPONENTS gps imgproc navigation video)
find_package(BoBThirdParty REQUIRED googletest)
find_package(Eigen3 REQUIRED)

add_executable(tests circstat.cc dct.cc differencers.cc geometry.cc
               gps_reader.cc image_database.cc infomax.cc mask.cc
               nmea_parser.cc opencv_unwrap_360_serialisation.cc
               perfect_memory.cc string.cc tests.cc)
target_include_directories(tests PUBLIC ${GTEST_INCLUDE_DIRS})
target_link_libraries(tests PUBLIC ${BoBRobotics_LIBRARIES}
                      ${BoBThirdParty_LIBRARIES} Eigen3::Eigen)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(tests PUBLIC OpenMP::OpenMP_CXX)
endif()
