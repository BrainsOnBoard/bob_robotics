
file(GLOB DLL_FILES "${CMAKE_CURRENT_BINARY_DIR}/*.dll")
file(COPY ${DLL_FILES} DESTINATION ../cmake-install/antworld)
