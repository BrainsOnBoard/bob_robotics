# The last known working commit for the ODK2 devkit is 4af04b8
if (NOT DEFINED ENV{ODK2_PATH})
    message(FATAL_ERROR "ODK2_PATH environment variable must be set to point to the development_kit_odk2 repo")
endif()
add_subdirectory("$ENV{ODK2_PATH}/devkit_driver"
                 "${CMAKE_CURRENT_BINARY_DIR}/BoB/third_party/${MODULE}")
set(LIBRARIES devkit_driver)
