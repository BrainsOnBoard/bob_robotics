# Extra path hint needed on Windows
if (WIN32)
    set(ENV{TBB_ROOT} "${VCPKG_PACKAGE_DIR}")
endif()

BoB_find_package(TBB REQUIRED)
