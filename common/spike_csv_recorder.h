#pragma once

// Standard C++ includes
#include <fstream>
#include <iterator>
#include <list>
#include <vector>

//----------------------------------------------------------------------------
// SpikeRecorder
//----------------------------------------------------------------------------
// Interface for spike recorders
class SpikeRecorder
{
public:
    virtual ~SpikeRecorder()
    {
    }

    //----------------------------------------------------------------------------
    // Declared virtuals
    //----------------------------------------------------------------------------
    virtual void record(double t) = 0;
};

//----------------------------------------------------------------------------
// SpikeCSVRecorder
//----------------------------------------------------------------------------
class SpikeCSVRecorder : public SpikeRecorder
{
public:
    SpikeCSVRecorder(const char *filename,  unsigned int *spkCnt, unsigned int *spk)
    : m_Stream(filename), m_SpkCnt(spkCnt), m_Spk(spk)
    {
        m_Stream << "Time [ms], Neuron ID" << std::endl;
    }

    //----------------------------------------------------------------------------
    // SpikeRecorder virtuals
    //----------------------------------------------------------------------------
    virtual void record(double t) override
    {
        for(unsigned int i = 0; i < m_SpkCnt[0]; i++)
        {
            m_Stream << t << "," << m_Spk[i] << std::endl;
        }
    }

private:

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::ofstream m_Stream;
    unsigned int *m_SpkCnt;
    unsigned int *m_Spk;
};

//----------------------------------------------------------------------------
// SpikeCSVRecorder
//----------------------------------------------------------------------------
class SpikeCSVRecorderCached : public SpikeRecorder
{
public:
    SpikeCSVRecorderCached(const char *filename,  unsigned int *spkCnt, unsigned int *spk)
    : m_Stream(filename), m_SpkCnt(spkCnt), m_Spk(spk)
    {
        m_Stream << "Time [ms], Neuron ID" << std::endl;
    }

    virtual ~SpikeCSVRecorderCached()
    {
        writeCache();
    }

    //----------------------------------------------------------------------------
    // SpikeRecorder virtuals
    //----------------------------------------------------------------------------
    virtual void record(double t) override
    {
        // Add a new entry to the cache
        m_Cache.emplace_back();

        // Fill in time
        m_Cache.back().first = t;

        // Reserve vector to hold spikes
        m_Cache.back().second.reserve(m_SpkCnt[0]);

        // Copy spikes into vector
        std::copy_n(m_Spk, m_SpkCnt[0], std::back_inserter(m_Cache.back().second));
    }

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void writeCache()
    {
        // Loop through timesteps
        for(const auto &timestep : m_Cache) {
            // Loop through spikes
            for(unsigned int spike : timestep.second) {
                // Write CSV
                m_Stream << timestep.first << "," << spike << std::endl;
            }
        }

        // Clear cache
        m_Cache.clear();
    }

private:
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::ofstream m_Stream;
    unsigned int *m_SpkCnt;
    unsigned int *m_Spk;

    std::list<std::pair<double, std::vector<unsigned int>>> m_Cache;
};

//----------------------------------------------------------------------------
// SpikeCSVRecorderDelay
//----------------------------------------------------------------------------
class SpikeCSVRecorderDelay : public SpikeRecorder
{
public:
    SpikeCSVRecorderDelay(const char *filename, unsigned int popSize, unsigned int &spkQueuePtr, unsigned int *spkCnt, unsigned int *spk)
    : m_Stream(filename), m_SpkQueuePtr(spkQueuePtr), m_SpkCnt(spkCnt), m_Spk(spk), m_PopSize(popSize)
    {
        m_Stream << "Time [ms], Neuron ID" << std::endl;
    }

    //----------------------------------------------------------------------------
    // SpikeRecorder virtuals
    //----------------------------------------------------------------------------
    virtual void record(double t) override
    {
        unsigned int *currentSpk = getCurrentSpk();
        for(unsigned int i = 0; i < getCurrentSpkCnt(); i++)
        {
            m_Stream << t << "," << currentSpk[i] << std::endl;
        }
    }

private:
    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    unsigned int *getCurrentSpk() const
    {
        return &m_Spk[m_SpkQueuePtr * m_PopSize];
    }

    unsigned int getCurrentSpkCnt() const
    {
        return m_SpkCnt[m_SpkQueuePtr];
    }
    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    std::ofstream m_Stream;
    unsigned int &m_SpkQueuePtr;
    unsigned int *m_SpkCnt;
    unsigned int *m_Spk;
    unsigned int m_PopSize;
};