#pragma once

/*
 * Check for different robot types by looking at base classes (e.g. all tanks
 * inherit from TankBase).
 */
#define BOB_DEFINE_TRAIT(TYPE)                                                        \
    namespace TYPE {                                                                  \
    template<class Derived>                                                           \
    class TYPE##Base;                                                                 \
    }                                                                                 \
    template<class T>                                                                 \
    struct Is##TYPE                                                                   \
    {                                                                                 \
        static constexpr bool value = std::is_base_of<TYPE::TYPE##Base<T>, T>::value; \
    };

namespace BoBRobotics {
namespace Robots {

BOB_DEFINE_TRAIT(Ackermann)
BOB_DEFINE_TRAIT(Tank)
BOB_DEFINE_TRAIT(Omni2D)
BOB_DEFINE_TRAIT(UAV)

} // Robots
} // BoBRobotics
