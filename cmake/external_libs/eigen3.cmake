if(UNIX)
    BoB_add_pkg_config_libraries(eigen3)
elseif(TARGET Eigen3::Eigen)
    BoB_add_link_libraries(Eigen3::Eigen)
else()
    message(FATAL_ERROR "Eigen 3 not found")
endif()

BoB_external_libraries(openmp)
