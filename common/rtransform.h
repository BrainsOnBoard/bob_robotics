#pragma once

// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
template<typename Range1, typename Range2, typename UnaryOp>
inline void rtransform(const Range1 &inrange, Range2 &outrange, UnaryOp func)
{
    std::transform(std::begin(inrange), std::end(inrange), std::begin(outrange), func);
}

template<typename Range, typename UnaryOp>
inline void
rtransform(Range &r, UnaryOp func)
{
    rtransform(r, r, func);
}

template<typename Range1, typename Range2, typename Range3, typename BinaryOp>
inline void rtransform(const Range1 &inrange1, const Range2 &inrange2, Range3 &outrange, BinaryOp func)
{
    std::transform(std::begin(inrange1), std::end(inrange1), std::begin(inrange2), std::begin(outrange), func);
}
} // BoBRobotics