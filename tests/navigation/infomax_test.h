// BoB robotics includes
#define EXPOSE_INFOMAX_INTERNALS
#include "navigation/infomax.h"

namespace BoBRobotics {
namespace Navigation {
constexpr size_t NumTrainStepsToTest = 1;

class InfoMaxTest
  : public InfoMaxRotater<> {
public:
    InfoMaxTest(const cv::Size &unwrapRes)
      : InfoMaxRotater<>{ unwrapRes, 0.0001f,
                          Normalisation::None,
                          generateInitialWeights(unwrapRes.width * unwrapRes.height,
                                                 unwrapRes.width * unwrapRes.height + 1,
                                                 /*seed=*/42)}
    {}

    template<class... Ts>
    const auto getImageDifferences(Ts&&... args) const
    {
        const auto &diffs = InfoMaxRotater<>::getImageDifferences(std::forward<Ts>(args)...);

        /*
         * Casting away the const is a bit gross, but Eigen requires a non-const
         * pointer here. It is safe as we are returning a const object.
         */
        return Eigen::Map<Eigen::VectorXf>{ const_cast<float *>(diffs.data()),
                                            static_cast<Eigen::Index>(diffs.size()) };
    }
}; // InfoMaxTest
} // Navigation
} // BoBRobotics
