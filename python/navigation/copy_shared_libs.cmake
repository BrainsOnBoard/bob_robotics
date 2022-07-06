# Copy shared library files to somewhere that the build system can find
set(DEST "${CMAKE_CURRENT_BINARY_DIR}/../../../bob_robotics/navigation")
foreach(EXT so dylib pyd dll)
    file(GLOB SHARED_LIBS "${CMAKE_CURRENT_BINARY_DIR}/*.${EXT}")
    file(COPY ${SHARED_LIBS} DESTINATION "${DEST}")
endforeach(EXT so dylib pyd dll)
