#pragma once

// BoB robotics includes
#include "common/macros.h"

// Third-party includes
#include "third_party/units.h"

// Standard C++ includes
#include <algorithm>
#include <initializer_list>
#include <type_traits>

namespace BoBRobotics {
template<class T = int>
class Range
{
public:
    class Iterator
    {
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = ssize_t;
        using pointer = T*;
        using reference = T&;

        Iterator(T value, T step = T{ 1 })
          : m_Step{ step }
          , m_Value{ value }
        {}

        auto &operator++()
        {
            m_Value += m_Step;
            return *this;
        }

        T operator*() const
        {
            return m_Value;
        }

        bool operator<(const Iterator &it) const
        {
            return m_Value < it.m_Value;
        }

        bool operator==(const Iterator &it) const
        {
            return m_Value == it.m_Value;
        }

        bool operator!=(const Iterator &it) const
        {
            return m_Value != it.m_Value;
        }

        ssize_t operator-(const Iterator &it) const
        {
            BOB_ASSERT(m_Step == it.m_Step);
            return static_cast<ssize_t>((m_Value - it.m_Value) / m_Step);
        }

    private:
        const T m_Step;
        T m_Value;
    };

    Range(T begin, T end, T step = T{ 1 })
      : m_Begin{ begin }
      , m_End{ end }
      , m_Step{ step }
    {
        checkValues();
        capEnd();
    }

    Range(T end)
      : m_End{ end }
    {
        checkValues();
    }

    Range()
      : m_Step{ 0 }
    {}

    Range(std::initializer_list<T> l)
    {
        BOB_ASSERT(l.size() < 4);

        switch(l.size()) {
        case 0:
            m_Step = T{ 0 };
            break;
        case 1:
            m_End = *l.begin();
            break;
        case 2:
            m_Begin = *l.begin();
            m_End = *(l.begin() + 1);
            break;
        case 3:
            m_Begin = *l.begin();
            m_End = *(l.begin() + 1);
            m_Step = *(l.begin() + 2);
        }

        checkValues();
        capEnd();
    }

    auto begin() const
    {
        return Iterator{ m_Begin, m_Step };
    }

    auto end() const
    {
        return Iterator{ m_End, m_Step };
    }

    T getStep() const
    {
        return m_Step;
    }

    size_t size() const
    {
        return static_cast<size_t>((m_End - m_Begin) / m_Step);
    }

    bool empty() const
    {
        return m_Begin == m_End;
    }

private:
    T m_Begin{ 0 }, m_End{ 0 }, m_Step{ 1 };

    template<typename U = T,
             typename std::enable_if_t<!units::traits::is_unit_t<U>::value>* = nullptr>
    void capEnd()
    {
        T mod = (m_End - m_Begin) % m_Step;
        if (mod != T{ 0 }) {
            m_End += m_Step - mod;
        }
    }

    template<typename U = T,
             typename std::enable_if_t<units::traits::is_unit_t<U>::value>* = nullptr>
    void capEnd()
    {
        auto div = (m_End - m_Begin) / m_Step;
        auto lower = units::math::floor(div);
        if (lower != div) {
            m_End = m_Begin + m_Step * (lower + 1);
        }
    }

    void checkValues() const
    {
        if (m_Begin < m_End) {
            BOB_ASSERT(m_Step > T{ 0 });
        } else if (m_Begin > m_End) {
            BOB_ASSERT(m_Step < T{ 0 });
        } else {
            BOB_ASSERT(m_Step == T{ 0 });
        }
    }
}; // Range

template<class T, class... Ts>
inline auto createRange(T arg, Ts&& ...args)
{
    return Range<T>{ arg, std::forward<Ts>(args)... };
}

inline auto createRange()
{
    return Range<>{};
}
} // BoBRobotics
