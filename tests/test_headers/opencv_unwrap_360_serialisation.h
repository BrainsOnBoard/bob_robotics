#include "common.h"

// BoB robotics includes
#include "imgproc/opencv_unwrap_360.h"

TEST(OpenCVUnwrap360, Serialisation) {
    const cv::Size cameraResolution(1280, 400);
    const cv::Size unwrapResolution(180, 50);
    BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper1(cameraResolution,
                                                     unwrapResolution,
                                                     0.45468750000000002,
                                                     0.20499999999999999,
                                                     0.087499999999999994,
                                                     0.19,
                                                     0_deg,
                                                     true);

    // Serialise data
    cv::FileStorage fsWrite(".yaml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
    fsWrite << "unwrapper" << unwrapper1;
    const std::string yamlStr = fsWrite.releaseAndGetString();

    // Deserialise data
    BoBRobotics::ImgProc::OpenCVUnwrap360 unwrapper2(cameraResolution, unwrapResolution);
    cv::FileStorage fsRead(yamlStr, cv::FileStorage::READ | cv::FileStorage::MEMORY);
    fsRead["unwrapper"] >> unwrapper2;

    // Compare
    EXPECT_EQ(unwrapper1.m_CentrePixel, unwrapper2.m_CentrePixel);
    EXPECT_EQ(unwrapper1.m_InnerPixel, unwrapper2.m_InnerPixel);
    EXPECT_EQ(unwrapper1.m_OuterPixel, unwrapper2.m_OuterPixel);
    EXPECT_EQ(unwrapper1.m_Flip, unwrapper2.m_Flip);
    BOB_EXPECT_UNIT_T_EQ(unwrapper1.m_OffsetAngle, unwrapper2.m_OffsetAngle);
}

