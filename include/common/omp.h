#pragma once

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
namespace OMP {

//! Returns maximum number of threads for OpenMP or 1 if OpenMP is not in use
size_t maxThreads();

//! Returns current thread ID or 0 if OpenMP is not in use
size_t currentThread();

//------------------------------------------------------------------------
// BoBRobotics::OMP::Array
//------------------------------------------------------------------------
/*!
 * \brief A wrapper providing thread-local storage for e.g. scratch data
 */
template<class T>
class Array {
public:
    //! Default initialise elements of array
    Array()
      : m_Data(maxThreads())
    {}

    //! Initialise elements of the array with the specified arguments
    template<class... Ts>
    Array(Ts &&... args)
    {
        const size_t threads = maxThreads();
        m_Data.reserve(threads);
        for (size_t i = 0; i < threads; i++) {
            m_Data.emplace_back(std::forward<Ts>(args)...);
        }
    }

    auto &elements() { return m_Data; }

    //! Get a reference to the current thread's scratch data
    auto &current() { return m_Data[currentThread()]; }
    const auto &current() const { return m_Data[currentThread()]; }

private:
    std::vector<T> m_Data;
};
} // OMP
} // BoBRobotics
