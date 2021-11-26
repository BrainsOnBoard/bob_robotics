cmake_minimum_required(VERSION 3.10)
include(../../cmake/BoBRobotics.cmake)
project(ardin_mb)

set(IS_EXPERIMENT TRUE)
find_package(BoBRobotics QUIET REQUIRED COMPONENTS antworld common navigation)
find_package(BoBThirdParty REQUIRED imgui)

if(NOT DEFINED NO_GENN)
    find_package(GeNN)
    if(GeNN_FOUND)
        add_genn_model(mb_memory_ardin model_ardin.cc)
    else()
        message(WARNING "GeNN not found; building without")
    endif()
endif()

set(SOURCES ardin_mb.cc state_handler.cc opencv_texture.cc vector_field.cc
    visual_navigation_ui.cc)

if(TARGET mb_memory_ardin)
    add_executable(ardin_mb ${SOURCES} mb_memory.cc mb_memory_ardin.cc)
    target_link_libraries(ardin_mb ${BoBRobotics_LIBRARIES} BoBThirdParty::imgui
                          mb_memory_ardin)
else()
    add_executable(ardin_mb ${SORUCES})
    target_link_libraries(ardin_mb ${BoBRobotics_LIBRARIES} BoBThirdParty::imgui)
endif()

if(UNIX)
    configure_file(run_benchmark.sh.in "${CMAKE_CURRENT_BINARY_DIR}/run_benchmark.sh")
    execute_process(COMMAND chmod +x run_benchmark.sh)
endif()
