#include "../navigation/generate_images.h"
#include "../navigation/PerfectMemoryRotater__.h"
#include "common.h"

// BoB robotics includes
#include "navigation/perfect_memory.h"

TEST(PerfectMemory, AllDifferences)
{
    generateImages();

    Navigation::PerfectMemoryRotater<> pm{ TestImageSize };
    for (const auto &image : TestImages) {
        pm.train(image);
    }

    const auto &differences = pm.getImageDifferences(TestImages[0]);
    for (size_t snap = 0; snap < NumTestImages; snap++) {
        for (size_t col = 0; col < (size_t) TestImageSize.width; col++) {
            EXPECT_FLOAT_EQ(differences[snap][col], trueDifferences[snap][col]);
        }
    }
}
