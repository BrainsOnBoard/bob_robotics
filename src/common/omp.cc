// BoB robotics includes
#include "common/omp.h"

// OpenMP
#include <omp.h>

namespace BoBRobotics {
namespace OMP {
size_t
maxThreads()
{
#ifdef _OPENMP
    return (size_t) omp_get_max_threads();
#else
    return 1;
#endif
}

size_t
currentThread()
{
#ifdef _OPENMP
    return (size_t) omp_get_thread_num();
#else
    return 0;
#endif
}
}
}
