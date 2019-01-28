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
#ifdef _WIN32
    } catch (std::exception &e) {
        // Windows doesn't print exception details by default
        std::cerr << "Uncaught exception: " << e.what() << std::endl;
#ifdef _DEBUG
        throw;
#else
        // There's no point telling Windows the program has crashed
        return EXIT_FAILURE;
#endif // !_DEBUG
#endif // _WIN32
    } catch (...) {
        // Rethrow the caught exception
        throw;
    }
}
