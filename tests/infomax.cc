#include "common.h"
#include "test_algo.h"
#include "navigation/infomax_test.h"

TEST(InfoMax, SampleImage)
{
    testAlgo<BoBRobotics::Navigation::InfoMaxTest>("infomax.bin");
}

// Check that the columns have means of approx 0 and SDs of approx 1
TEST(InfoMax, RandomWeightsDistribution)
{
    constexpr float Precision = 1e-6f;

    const auto weights = BoBRobotics::Navigation::InfoMax<>::generateInitialWeights(100, 100, /*seed=*/42);
    const auto means = weights.colwise().mean();
    for (int i = 0; i < means.size(); i++) {
        EXPECT_NEAR(means[i], 0.f, Precision);
    }

    const auto stds = (weights.array() * weights.array()).colwise().mean().sqrt();
    for (int i = 0; i < stds.size(); i++) {
        EXPECT_NEAR(stds[i], 1.f, Precision);
    }
}
