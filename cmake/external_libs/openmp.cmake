if(OpenMP_CXX_FOUND)
    BoB_add_link_libraries(OpenMP::OpenMP_CXX)
else()
    # Let's not make it a fatal error if we can't find it
    message(WARNING "Could not find OpenMP; code will be single threaded")
endif()
