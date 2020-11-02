#pragma once

namespace BoBRobotics {
namespace Navigation {
template<class Functor, class BaseAlgorithm>
class Preprocessor
  : BaseAlgorithm {
public:
    template<class T>
    void train(const T &image)
    {
        BaseAlgorithm::train(m_Functor(image));
    }

    template<class T>
    auto test(const T &image)
    {
        return BaseAlgorithm::test(m_Functor(image));
    }

private:
    Functor m_Functor;
}; // Preprocessor
} // Navigation
} // BoBRobotics
