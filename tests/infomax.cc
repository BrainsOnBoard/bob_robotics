#include "common.h"
#include "test_algo.h"
#include "navigation/generate_images.h"
#include "navigation/infomax_test.h"

using namespace BoBRobotics::Navigation;

const auto InitialWeights = InfoMax<>::generateInitialWeights(TestImageSize.width * TestImageSize.height, 100, /*seed=*/42);

TEST(InfoMax, SampleImage)
{
    testAlgo<InfoMaxTest>("infomax.bin");
}

// Check that the columns have means of approx 0 and SDs of approx 1
TEST(InfoMax, RandomWeightsDistribution)
{
    constexpr float Precision = 1e-6f;

    const auto means = InitialWeights.colwise().mean();
    for (int i = 0; i < means.size(); i++) {
        EXPECT_NEAR(means[i], 0.f, Precision);
    }

    const auto stds = (InitialWeights.array() * InitialWeights.array()).colwise().mean().sqrt();
    for (int i = 0; i < stds.size(); i++) {
        EXPECT_NEAR(stds[i], 1.f, Precision);
    }
}

// Check that too high a learning rate causes weights to blow up
TEST(InfoMax, ExplodingWeights)
{
    InfoMaxRotater<> infomax{ TestImageSize, InitialWeights, /*learningRate=*/0.1f };

    EXPECT_THROW({
        for (const auto &image : TestImages) {
            infomax.train(image);
        }
    }, WeightsBlewUpError);
}

// Check that using a sensible learning rate doesn't throw an exception
TEST(InfoMax, NonExplodingWeights)
{
    InfoMaxRotater<> infomax{ TestImageSize, InitialWeights, /*learningRate=*/1e-5f };

    EXPECT_NO_THROW({
        for (const auto &image : TestImages) {
            infomax.train(image);
        }
    });
}
