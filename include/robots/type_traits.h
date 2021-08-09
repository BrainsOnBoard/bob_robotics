#pragma once

#define BOB_DEFINE_HAS_METHOD(traitName, methodName)      \
    template<class T>                                     \
    class traitName                                       \
    {                                                     \
    private:                                              \
        typedef char YesType[1];                          \
        typedef char NoType[2];                           \
                                                          \
        template<class C>                                 \
        static YesType &test(decltype(&C::methodName));   \
        template<class C>                                 \
        static NoType &test(...);                         \
                                                          \
    public:                                               \
        enum                                              \
        {                                                 \
            value = sizeof(test<T>(0)) == sizeof(YesType) \
        };                                                \
    }

namespace BoBRobotics
{
namespace Robots
{
BOB_DEFINE_HAS_METHOD(isTank, tank);
BOB_DEFINE_HAS_METHOD(isOmni2D, omni2d);
BOB_DEFINE_HAS_METHOD(hasMove, move);
BOB_DEFINE_HAS_METHOD(isUAV, setVerticalSpeed);

// Tank robots may also have a move method, so let's disambiguate that case
template<class T>
class isAckermann
{
public:
    enum
    {
        value = hasMove<T>::value && !isTank<T>::value
    };
};
} // Robots
} // BoBRobotics
