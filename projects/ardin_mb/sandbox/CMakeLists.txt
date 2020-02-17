cmake_minimum_required(VERSION 2.8)
project( WaveletSandbox )
find_package( OpenCV REQUIRED )
link_directories(${CMAKE_SOURCE_DIR}/wavelib)
add_executable( WaveletSandbox WaveletSandbox.cpp )
 
target_link_libraries( WaveletSandbox ${OpenCV_LIBS} 
	libwavelet2s.a)
#include_directories(${CMAKE_SOURCE_DIR}/wavelib)
# add the binary tree to the search path for include files
# so that we will find TutorialConfig.h

