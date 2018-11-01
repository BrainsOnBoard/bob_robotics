/*
 * This header provides a simple wrapper interface for the common use-case where
 * you want to catch all exceptions thrown in the main routine in order to do
 * appropriate clean-up (esp. stopping robots moving etc.).
 *
 * This is also handy on Windows where there is no default exception handler, in
 * order to see what exceptions have actually been thrown.
 */

#pragma once

// Standard C++ includes
#include <exception>
#include <iostream>

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
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
#ifndef DEBUG
        // There's no point telling Windows the program has crashed
        return EXIT_FAILURE;
#endif // !DEBUG
#endif // _WIN32

        // Rethrow the caught exception
        throw;
    }
}
