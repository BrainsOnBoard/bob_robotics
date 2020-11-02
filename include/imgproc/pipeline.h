#pragma once

// Standard C++ includes
#include <tuple>
#include <type_traits>
#include <utility>

// Standard C includes
#include <cstddef>

namespace BoBRobotics {
namespace ImgProc {
template<class... Funcs>
class Pipeline {
public:
    template<class... Ts>
    Pipeline(Ts&&... args)
      : m_Functors(std::forward<Ts>(args)...)
    {}

    template<class T>
    auto operator()(const T &val) const
    {
        return apply<0>(val);
    }

private:
    std::tuple<Funcs...> m_Functors;

    template<size_t I, class T, std::enable_if_t<I < sizeof...(Funcs), int> = 0>
    constexpr auto apply(T &&arg) const
    {
        return apply<I+1>(std::get<I>(m_Functors)(arg));
    }

    template<size_t I, class T, std::enable_if_t<I == sizeof...(Funcs), int> = 0>
    static constexpr auto apply(T &&arg)
    {
        return arg;
    }
}; // Pipeline

template<class... Funcs>
auto createPipeline(Funcs&&... functors)
{
    return Pipeline<Funcs...>(std::forward<Funcs>(functors)...);
}
} // ImgProc
} // BoBRobotics
