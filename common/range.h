// Standard C++ includes
#include <algorithm>

namespace BoBRobotics {
template<typename Range1, typename Range2, typename UnaryOp>
inline void transform(Range1 &&r1, Range2 &&r2, UnaryOp &&func)
{
    std::transform(std::begin(r1), std::end(r1), std::begin(r2), func);
}

template<typename Range, typename UnaryOp>
inline void transform(Range &&r, UnaryOp &&func)
{
    transform(r, r, func);
}
}