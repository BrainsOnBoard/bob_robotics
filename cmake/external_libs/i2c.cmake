if(${lib} STREQUAL i2c)
    if(NOT WIN32 AND NOT NO_I2C)
        # If it's a new version of i2c-tools then we need to link
        # against an additonal library
        execute_process(COMMAND "${BOB_ROBOTICS_PATH}/bin/is_i2c_tools_new.sh"
                        RESULT_VARIABLE rv)
        if(${rv} EQUAL 0)
            BoB_add_link_libraries("i2c")
        endif()
    endif()
endif()
