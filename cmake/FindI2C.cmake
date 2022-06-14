set(_I2C_REQUIRED_VARS I2C_VERSION)

find_program(I2CDETECT i2cdetect)
if(I2CDETECT)
    execute_process(COMMAND "${I2CDETECT}" -V ERROR_VARIABLE EV
                    ERROR_STRIP_TRAILING_WHITESPACE)
    string(REGEX MATCH "[^ ]+$" I2C_VERSION "${EV}")
    if(I2C_VERSION AND I2C_VERSION VERSION_GREATER_EQUAL 4)
        find_library(I2C_LIBRARIES i2c)
        list(APPEND _I2C_REQUIRED_VARS I2C_LIBRARIES)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(I2C REQUIRED_VARS ${_I2C_REQUIRED_VARS}
                                  VERSION_VAR I2C_VERSION)
