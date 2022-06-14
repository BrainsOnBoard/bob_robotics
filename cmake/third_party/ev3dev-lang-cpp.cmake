# Default to BrickPi3
if(NOT EV3DEV_PLATFORM)
    set(EV3DEV_PLATFORM "BRICKPI3" CACHE STRING "Target ev3dev platform (EV3/BRICKPI/BRICKPI3/PISTORMS)")
endif()
message(STATUS "EV3 platform: ${EV3DEV_PLATFORM}")

set_property(CACHE EV3DEV_PLATFORM PROPERTY STRINGS "EV3" "BRICKPI" "BRICKPI3" "PISTORMS")
list(APPEND DEFINITIONS -DEV3DEV_PLATFORM_${EV3DEV_PLATFORM})

# Some things in the ev3dev library are not properly marked as inline or put
# into .cpp files, so they cause linking errors on newer versions of gcc.
list(APPEND LIBRARIES -Wl,--allow-multiple-definition ev3dev)
