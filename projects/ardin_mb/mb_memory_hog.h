#pragma once

// Standard C++ includes
#include <array>
#include <bitset>
#include <list>
#include <random>
#include <tuple>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>

// BoB robotics includes
#include "genn_utils/shared_library_model.h"
#include "navigation/visual_navigation_base.h"

// Ardin MB includes
#include "mb_params_hog.h"

// Forward declarations
namespace CLI
{
class App;
}

//----------------------------------------------------------------------------
// MBMemoryHOG
//----------------------------------------------------------------------------
class MBMemoryHOG : public BoBRobotics::Navigation::VisualNavigationBase
{
public:
    MBMemoryHOG();
    virtual ~MBMemoryHOG();

    //------------------------------------------------------------------------
    // Typedefines
    //------------------------------------------------------------------------
    typedef std::list<std::pair<double, std::vector<unsigned int>>> Spikes;

    //------------------------------------------------------------------------
    // VisualNavigationBase virtuals
    //------------------------------------------------------------------------
    //! Train the algorithm with the specified image
    virtual void train(const cv::Mat &image) override;

    //! Test the algorithm with the specified image
    virtual float test(const cv::Mat &image) const override;

    //! Clear the memory
    virtual void clearMemory() override;

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    // **NOTE** gross API but allows easy integration with UI
    float *getGGNToKCWeight(){ return m_GGGNToKC; }
    float *getKCToGGNWeight(){ return m_GKCToGGN; }
    float *getKCToENWeight(){ return &m_KCToENWeight; };
    float *getPNToKC(){ return m_GPNToKC; }

    float *getGGNToKCVMid(){ return m_VMidGGNToKC; }
    float *getGGNToKCVslope(){ return m_VSlopeGGNToKC; }
    float *getGGNToKCVthresh(){ return m_VThreshGGNToKC; }

    int *getKCToENSynapse(){ return &m_KCToENSynape; }
    float *getKCToENDopamineStrength(){ return &m_KCToENDopamineStrength; }

    float *getPNInputCurrentScale(){ return m_IExtScalePN; }
    float *getPNVthresh(){ return m_VThreshPN; }
    float *getPNTauM(){ return &m_PNTauM; }
    float *getPNC(){ return &m_PNC; }

    float *getPNToKCTauSyn(){ return &m_PNToKCTauSyn; }

    const cv::Mat &getHOGFeatures() const{ return m_HOGFeatures; }

    const Spikes &getPNSpikes() const{ return m_PNSpikes; }
    const Spikes &getKCSpikes() const{ return m_KCSpikes; }
    const Spikes &getENSpikes() const{ return m_ENSpikes; }

    unsigned int getNumPNSpikes() const{ return m_NumPNSpikes; }
    unsigned int getNumKCSpikes() const{ return m_NumKCSpikes; }
    unsigned int getNumENSpikes() const{ return m_NumENSpikes; }

    unsigned int getNumUnusedWeights() const{ return m_NumUsedWeights; }
    unsigned int getNumActivePN() const{ return m_NumActivePN; }
    unsigned int getNumActiveKC() const{ return m_NumActiveKC; }

    const std::vector<float> &getGGNVoltageHistory() const{ return m_GGNVoltageHistory; }
    const std::vector<float> &getKCInhInSynHistory() const{ return m_KCInhInSynHistory; }

    const std::vector<float> &getDKCToENHistory() const{ return m_DKCToENHistory; }
    const std::vector<float> &getCKCToENHistory() const{ return m_CKCToENHistory; }
    const std::vector<float> &getGKCToENHistory() const{ return m_GKCToENHistory; }

    const std::vector<float> &getENVoltageHistory() const{ return m_ENVoltageHistory; }

    float *getRewardTimeMs(){ return &m_RewardTimeMs; }
    float *getPresentDurationMs(){ return &m_PresentDurationMs; }

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode &node);

    void addCLIArguments(CLI::App &app);

    const std::array<cv::Vec2f, MBParamsHOG::hogNumOrientations> &getHOGDirections() const{ return m_HOGDirections; }

private:
    //------------------------------------------------------------------------
    // Private API
    //------------------------------------------------------------------------
    void setInput(const std::vector<float> &input);
    std::tuple<unsigned int, unsigned int, unsigned int> present(const cv::Mat &image, bool train) const;

    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    mutable cv::Mat m_SnapshotFloat;

    // Sobel operator output
    mutable cv::Mat m_SobelX;
    mutable cv::Mat m_SobelY;

    // Image containing pixel orientations - one channel per orientation
    mutable cv::Mat m_PixelOrientations;

    mutable cv::Mat m_HOGFeatures;

    float m_RateScalePN;

    float m_PNToKCTauSyn;
    float m_PNTauM;
    float m_PNC;

    float m_KCToENWeight;
    float m_KCToENDopamineStrength;

    float m_RewardTimeMs;
    float m_PresentDurationMs;

    mutable Spikes m_PNSpikes;
    mutable Spikes m_KCSpikes;
    mutable Spikes m_ENSpikes;

    mutable unsigned int m_NumPNSpikes;
    mutable unsigned int m_NumKCSpikes;
    mutable unsigned int m_NumENSpikes;

    mutable unsigned int m_NumUsedWeights;
    mutable unsigned int m_NumActivePN;
    mutable unsigned int m_NumActiveKC;

    mutable std::vector<float> m_GGNVoltageHistory;
    mutable std::vector<float> m_KCInhInSynHistory;

    int m_KCToENSynape;
    mutable std::vector<float> m_DKCToENHistory;
    mutable std::vector<float> m_CKCToENHistory;
    mutable std::vector<float> m_GKCToENHistory;

    mutable std::vector<float> m_ENVoltageHistory;

    mutable std::mt19937 m_RNG;

    std::array<cv::Vec2f, MBParamsHOG::hogNumOrientations> m_HOGDirections;

    mutable BoBRobotics::GeNNUtils::SharedLibraryModelFloat m_SLM;

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
    float *m_TDKCToEN;
    float *m_TCKCToEN;
    float *m_DKCToEN;
    bool *m_InjectDopamineKCToEN;
    unsigned int *m_SpkCntEN;
    unsigned int *m_SpkEN;
    unsigned int *m_SpkCntKC;
    unsigned int *m_SpkKC;
    unsigned int *m_SpkCntPN;
    unsigned int *m_SpkPN;
    float *m_VGGN;
    float *m_InSynGGNToKC;
    float *m_CKCToEN;
    float *m_GKCToEN;
    float *m_VEN;

};
