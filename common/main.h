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
#ifdef DEBUG
    /*
     * If we're debugging, we don't catch exceptions, so that a debugger can
     * catch them.
     */
    return bob_main(argc, argv);
#else
#ifdef _WIN32
    try {
        return bob_main(argc, argv);
    } catch (std::exception &e) {
        // Windows doesn't print exception details by default
        std::cerr << "Uncaught exception: " << e.what() << std::endl;

        // There's no point telling Windows the program has crashed
        return EXIT_FAILURE;
    }
#else
    // On *nix, use default exception handler, after clean-up
    try {
        return bob_main(argc, argv);
    } catch (...) {
        throw;
    }
#endif // _WIN32
#endif // DEBUG
}
