add_subdirectory("${BOB_ROBOTICS_PATH}/third_party/development_kit_odk2/devkit_driver"
                 "${CMAKE_CURRENT_BINARY_DIR}/BoB/third_party/${MODULE}")
set(LIBRARIES devkit_driver)
