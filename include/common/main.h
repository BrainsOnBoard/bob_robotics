/*
 * This header provides a simple wrapper interface for the common use-case where
 * you want to catch all exceptions thrown in the main routine in order to do
 * appropriate clean-up (esp. stopping robots moving etc.).
 *
 * This is also handy on Windows where there is no default exception handler, in
 * order to see what exceptions have actually been thrown.
 */

#pragma once

// BoB robotics includes
#include "common/logging.h"

// Standard C++ includes
#include <exception>

//! Add a definition for this function to your own main *.cc file
int
bob_main(int argc, char **argv);

int
main(int argc, char **argv)
{
    try {
        return bob_main(argc, argv);
    } catch (std::exception &e) {
#ifdef _WIN32
        // Windows doesn't print exception details by default
        LOG_FATAL << "Uncaught exception: " << e.what();
#endif

        // Rethrow exception so it can be handled by OS's default handler
        throw;
    } catch (...) {
        // Rethrow exceptions not of type std::exception
        throw;
    }
}
