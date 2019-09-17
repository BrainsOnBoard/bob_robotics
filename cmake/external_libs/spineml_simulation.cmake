# Find where user has installed GeNN
exec_or_fail("${BOB_ROBOTICS_PATH}/cmake/find_genn.sh")
string(STRIP "${SHELL_OUTPUT}" GENN_PATH) # Strip newline
message("GENN_PATH: ${GENN_PATH}")

BoB_add_include_directories("${GENN_PATH}/include")
BoB_add_link_libraries("${GENN_PATH}/lib/libspineml_simulator.a"
                        "${GENN_PATH}/lib/libspineml_common.a"
                        dl)
