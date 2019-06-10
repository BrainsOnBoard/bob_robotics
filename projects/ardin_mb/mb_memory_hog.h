#pragma once

// Standard C++ includes
#include <array>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// Ardin MB includes
#include "mb_memory.h"
#include "mb_params_hog.h"

//----------------------------------------------------------------------------
// MBMemoryHOG
//----------------------------------------------------------------------------
class MBMemoryHOG : public MBMemory
{
public:
    MBMemoryHOG();

    //------------------------------------------------------------------------
    // MBMemory virtuals
    //------------------------------------------------------------------------
    virtual void write(cv::FileStorage& fs) const override;
    virtual void read(const cv::FileNode &node) override;

    virtual void addCLIArguments(CLI::App &app) override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // **NOTE** gross API but allows easy integration with UI
    float *getGGNToKCWeight(){ return m_GGGNToKC; }
    float *getKCToGGNWeight(){ return m_GKCToGGN; }
    float *getPNToKC(){ return m_GPNToKC; }

    float *getGGNToKCVMid(){ return m_VMidGGNToKC; }
    float *getGGNToKCVslope(){ return m_VSlopeGGNToKC; }
    float *getGGNToKCVthresh(){ return m_VThreshGGNToKC; }

    float *getPNInputCurrentScale(){ return m_IExtScalePN; }
    float *getPNVthresh(){ return m_VThreshPN; }
    float *getPNTauM(){ return &m_PNTauM; }
    float *getPNC(){ return &m_PNC; }

    float *getPNToKCTauSyn(){ return &m_PNToKCTauSyn; }

    const cv::Mat &getFeatures() const{ return m_Features; }

    const std::vector<float> &getGGNVoltageHistory() const{ return m_GGNVoltageHistory; }
    const std::vector<float> &getKCInhInSynHistory() const{ return m_KCInhInSynHistory; }


    const std::array<cv::Vec2f, MBParamsHOG::numOrientations> &getDirections() const{ return m_Directions; }

protected:
    //------------------------------------------------------------------------
    // MBMemory virtuals
    //------------------------------------------------------------------------
    virtual void initPresent(unsigned long long duration) const override;
    virtual void beginPresent(const cv::Mat &snapshotFloat) const override;
    virtual void endPresent() const override;
    virtual void recordAdditional() const override;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    // Sobel operator output
    mutable cv::Mat m_SobelX;
    mutable cv::Mat m_SobelY;

    // Image containing pixel orientations - one channel per orientation
    mutable cv::Mat m_PixelOrientations;

    // Final feature vector to pass to MB
    mutable cv::Mat m_Features;

    float m_PNToKCTauSyn;
    float m_PNTauM;
    float m_PNC;

    mutable std::vector<float> m_GGNVoltageHistory;
    mutable std::vector<float> m_KCInhInSynHistory;

    // Vectors used to calculate orientation features from Sobel features
    std::array<cv::Vec2f, MBParamsHOG::numOrientations> m_Directions;

    float *m_GGGNToKC;
    float *m_GKCToGGN;
    float *m_GPNToKC;
    float *m_VMidGGNToKC;
    float *m_VSlopeGGNToKC;
    float *m_VThreshGGNToKC;
    float *m_IExtScalePN;
    float *m_VThreshPN;

    float *m_ExpTCPN;
    float *m_RmembranePN;
    float *m_ExpDecaypnToKC;
    float *m_InitPNToKC;
    float *m_IExtPN;

    float *m_VGGN;
    float *m_InSynGGNToKC;
    float *m_CKCToEN;
    float *m_VEN;

};
