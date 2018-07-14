/*
 * **NOTE**: This program must be run in release mode not debug mode in Visual Studio. Anaconda doesn't ship the debug versions of the required libs :-(
 */

#include "third_party/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int
main()
{
    // do some plotting :-)
    plt::plot({ 1, 2, 3, 4 });
    plt::show();

    return 0;
}
