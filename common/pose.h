#pragma once

// Third-party includes
#include "../third_party/units.h"

// Standard C++ includes
#include <array>

namespace BoBRobotics {

template<typename Derived>
class PoseBase
{
public:
    template<typename PoseType>
    bool operator==(const PoseType &pose) const
    {
        const auto derived = static_cast<const Derived *>(this);
        return derived->x() == pose.x() && derived->y() == pose.y() && derived->z() == pose.z()
                && derived->yaw() == pose.yaw() && derived->pitch() == pose.pitch() && derived->roll() == pose.roll();
    }

    template<typename PoseType>
    bool operator!=(const PoseType &pose) const
    {
        return !(*this == pose);
    }

    template<typename PositionType>
    auto distance2D(const PositionType &point) const
    {
        const auto derived = static_cast<const Derived *>(this);
        return derived->position().distance2D(point.position());
    }

    template<typename PositionType>
    auto distance3D(const PositionType &point) const
    {
        const auto derived = static_cast<const Derived *>(this);
        return derived->position().distance3D(point.position());
    }
};

//! Base class for vectors of length units
template<typename LengthUnit, size_t N, typename Derived>
class VectorBase
  : public PoseBase<Derived>
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    using radian_t = units::angle::radian_t;

public:
    VectorBase() = default;

    template<typename... Ts>
    VectorBase(Ts &&... args)
      : PoseBase<Derived>()
      , m_Array({ std::forward<Ts>(args)... })
    {}

    operator const std::array<LengthUnit, N> &() const
    {
        return m_Array;
    }

    template<typename PositionType>
    LengthUnit distance3D(const PositionType &point) const
    {
        const auto derived = static_cast<const Derived *>(this);
        using namespace units::math;
        return sqrt(pow<2>(derived->x() - point.x()) +
                    pow<2>(derived->y() - point.y()) +
                    pow<2>(derived->z() - point.z()));
    }

    template<typename PositionType>
    LengthUnit distance2D(const PositionType &point) const
    {
        const auto derived = static_cast<const Derived *>(this);
        using namespace units::math;
        return hypot(derived->x() - point.x(), derived->y() - point.y());
    }

    LengthUnit &operator[](size_t i) { return m_Array[i]; }
    const LengthUnit &operator[](size_t i) const { return m_Array[i]; }
    static constexpr size_t size() { return N; }

    const auto &position() const { return static_cast<const Derived &>(*this); }
    auto &position() { return static_cast<Derived &>(*this); }

    auto begin() { return m_Array.begin(); }
    auto begin() const { return m_Array.begin(); }
    auto end() { return m_Array.end(); }
    auto end() const { return m_Array.end(); }
    auto cbegin() const { return m_Array.cbegin(); }
    auto cend() const { return m_Array.end(); }

    static constexpr radian_t yaw() { return radian_t(0); }
    static constexpr radian_t pitch() { return radian_t(0); }
    static constexpr radian_t roll() { return radian_t(0); }

private:
    std::array<LengthUnit, N> m_Array{};
};

template<typename LengthUnit>
class Vector3;

//! 2D length unit vector
template<typename LengthUnit>
class Vector2
  : public VectorBase<LengthUnit, 2, Vector2<LengthUnit>>
{
public:
    Vector2() = default;

    Vector2(LengthUnit x, LengthUnit y)
      : VectorBase<LengthUnit, 2, Vector2<LengthUnit>>(x, y)
    {}

    Vector2(const std::array<LengthUnit, 2> &array)
      : Vector2(array[0], array[1])
    {}

    operator Vector3<LengthUnit>() const { return Vector3<LengthUnit>(x(), y(), z()); }

    LengthUnit &x() { return (*this)[0]; }
    const LengthUnit &x() const { return (*this)[0]; }
    LengthUnit &y() { return (*this)[1]; }
    const LengthUnit &y() const { return (*this)[1]; }
    static constexpr LengthUnit z() { return LengthUnit(0); }
};

//! 3D length unit vector
template<typename LengthUnit>
class Vector3
  : public VectorBase<LengthUnit, 3, Vector3<LengthUnit>>
{
public:
    Vector3() = default;

    Vector3(LengthUnit x, LengthUnit y, LengthUnit z)
      : VectorBase<LengthUnit, 3, Vector3<LengthUnit>>(x, y, z)
    {}

    Vector3(const std::array<LengthUnit, 3> &array)
      : Vector3(array[0], array[1], array[2])
    {}

    operator Vector2<LengthUnit>() const { return Vector2<LengthUnit>(x(), y()); }

    LengthUnit &x() { return (*this)[0]; }
    const LengthUnit &x() const { return (*this)[0]; }
    LengthUnit &y() { return (*this)[1]; }
    const LengthUnit &y() const { return (*this)[1]; }
    LengthUnit &z() { return (*this)[2]; }
    const LengthUnit &z() const { return (*this)[2]; }
};

// Forward declaration
template<typename LengthUnit, typename AngleUnit>
class Pose3;

//! A two-dimensional pose
template<typename LengthUnit, typename AngleUnit>
class Pose2
  : public PoseBase<Pose2<LengthUnit, AngleUnit>>
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    static_assert(units::traits::is_angle_unit<AngleUnit>::value,
                  "AngleUnit is not a unit of angle");

public:
    Pose2() = default;

    Pose2(LengthUnit x, LengthUnit y, AngleUnit angle)
      : m_Position{ x, y }
      , m_Angle(angle)
    {}

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose2<LengthUnit2, AngleUnit2>() const
    {
        return Pose2<LengthUnit2, AngleUnit2>{ x(), y(), yaw() };
    }

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose3<LengthUnit2, AngleUnit2>() const
    {
        return Pose3<LengthUnit2, AngleUnit2>{ { x(), y(), z() }, { yaw(), pitch(), roll() } };
    }

    Vector2<LengthUnit> &position() { return m_Position; }
    const Vector2<LengthUnit> &position() const { return m_Position; }
    LengthUnit &x() { return m_Position[0]; }
    const LengthUnit &x() const { return m_Position[0]; }
    LengthUnit &y() { return m_Position[1]; }
    const LengthUnit &y() const { return m_Position[1]; }
    static constexpr LengthUnit z() { return LengthUnit(0); }

    std::array<AngleUnit, 3> attitude() const { return { yaw(), AngleUnit(0), AngleUnit(0) }; }
    AngleUnit &yaw() { return m_Angle; }
    const AngleUnit &yaw() const { return m_Angle; }
    static constexpr AngleUnit pitch() { return AngleUnit(0); }
    static constexpr AngleUnit roll() { return AngleUnit(0); }

private:
    Vector2<LengthUnit> m_Position;
    AngleUnit m_Angle;
};

//! A three-dimensional pose
template<typename LengthUnit, typename AngleUnit>
class Pose3
  : public PoseBase<Pose3<LengthUnit, AngleUnit>>
{
    static_assert(units::traits::is_length_unit<LengthUnit>::value,
                  "LengthUnit is not a unit of length");
    static_assert(units::traits::is_angle_unit<AngleUnit>::value,
                  "AngleUnit is not a unit of angle");

public:
    Pose3() = default;

    Pose3(const Vector3<LengthUnit> &position, const std::array<AngleUnit, 3> &attitude)
      : m_Position(position)
      , m_Attitude(attitude)
    {}

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose2<LengthUnit2, AngleUnit2>() const
    {
        return Pose2<LengthUnit2, AngleUnit2>{ x(), y(), yaw() };
    }

    template<typename LengthUnit2, typename AngleUnit2>
    operator Pose3<LengthUnit2, AngleUnit2>() const
    {
        return Pose3<LengthUnit2, AngleUnit2>{ { x(), y(), z() }, { yaw(), pitch(), roll() } };
    }

    Vector3<LengthUnit> &position() { return m_Position; }
    const Vector3<LengthUnit> &position() const { return m_Position; }
    LengthUnit &x() { return m_Position[0]; }
    const LengthUnit &x() const { return m_Position[0]; }
    LengthUnit &y() { return m_Position[1]; }
    const LengthUnit &y() const { return m_Position[1]; }
    LengthUnit &z() { return m_Position[2]; }
    const LengthUnit &z() const { return m_Position[2]; }

    std::array<AngleUnit, 3> &attitude() { return m_Attitude; }
    const std::array<AngleUnit, 3> &attitude() const { return m_Attitude; }
    AngleUnit &yaw() { return m_Attitude[0]; }
    const AngleUnit &yaw() const { return m_Attitude[0]; }
    AngleUnit &pitch() { return m_Attitude[1]; }
    const AngleUnit &pitch() const { return m_Attitude[1]; }
    AngleUnit &roll() { return m_Attitude[2]; }
    const AngleUnit &roll() const { return m_Attitude[2]; }

private:
    Vector3<LengthUnit> m_Position;
    std::array<AngleUnit, 3> m_Attitude{};
};

//! Converts the input array to a unit-type of OutputUnit
template<typename OutputUnit, typename ArrayType>
inline constexpr std::array<OutputUnit, 3>
convertUnitArray(const ArrayType &values)
{
    return { static_cast<OutputUnit>(values[0]),
             static_cast<OutputUnit>(values[1]),
             static_cast<OutputUnit>(values[2]) };
}
} // BoBRobotics
