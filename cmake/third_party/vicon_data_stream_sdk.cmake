# This variable configures whether debug or release build is made
#set($ENV{CONFIG} ${CMAKE_BUILD_TYPE})
set(SDK_ROOT "${BOB_ROBOTICS_PATH}/third_party/vicon_data_stream_sdk")

message("Building Vicon SDK...")
exec_or_fail(make -C "${SDK_ROOT}" -f "${SDK_ROOT}/NoUnitTestingMakefile")

# Linking info
BoB_add_include_directories("${SDK_ROOT}/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP")
BoB_add_link_libraries("${SDK_ROOT}/bin/Debug/libViconDataStreamSDK_CPP.so")
