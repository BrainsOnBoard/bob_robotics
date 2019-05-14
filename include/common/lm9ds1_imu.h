#pragma once
#ifdef __linux__

// BoB robotics includes
#include "i2c_interface.h"

// Standard C includes
#include <cstdint>

namespace BoBRobotics {
//----------------------------------------------------------------------------
// BoBRobotics::LM9DS1
//----------------------------------------------------------------------------
//! Interface for the SparkFun 9DoF IMU
class LM9DS1
{
public:
    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    enum class Axis : uint8_t
    {
        X   = 0,
        Y   = 1,
        Z   = 2,
        All = 3,
    };

    enum class GyroScale : uint8_t
    {
        DPS245  = 0,
        DPS500  = 1,
        DPS2000 = 3,
    };

    enum class GyroSampleRate : uint8_t
    {
        Disabled    = 0,
        Hz14_9      = 1,
        Hz59_5      = 2,
        Hz119       = 3,
        Hz238       = 4,
        Hz476       = 5,
        Hz952       = 6,
    };

    enum class GyroHPF : uint8_t
    {
        Disabled    = 0,
        Cutoff0     = (1 << 6) | 0,
        Cutoff1     = (1 << 6) | 1,
        Cutoff2     = (1 << 6) | 2,
        Cutoff3     = (1 << 6) | 3,
        Cutoff4     = (1 << 6) | 4,
        Cutoff5     = (1 << 6) | 5,
        Cutoff6     = (1 << 6) | 6,
        Cutoff7     = (1 << 6) | 7,
        Cutoff8     = (1 << 6) | 8,
        Cutoff9     = (1 << 6) | 9,
    };

    enum class AccelScale : uint8_t
    {
        G2  = 0,
        G16 = 1,
        G4  = 2,
        G8  = 3,
    };

    enum class AccelSampleRate : uint8_t
    {
        Disabled    = 0,
        Hz10        = 1,
        Hz50        = 2,
        Hz119       = 3,
        Hz238       = 4,
        Hz476       = 5,
        Hz952       = 6,
    };

    enum class AccelBandwidth : uint8_t
    {
        DeterminedBySampleRate  = 0,
        Hz408                   = (1 << 2) | 0,
        Hz211                   = (1 << 2) | 1,
        Hz105                   = (1 << 2) | 2,
        Hz50                    = (1 << 2) | 3,
    };

    enum class AccelHighResBandwidth : uint8_t
    {
        Disabled    = 0,
        ODR50       = (1 << 7) | (0 << 5),
        ODR100      = (1 << 7) | (1 << 5),
        ODR9        = (1 << 7) | (2 << 5),
        ODR400      = (1 << 7) | (3 << 5),
    };

    enum class MagnetoScale : uint8_t
    {
        GS4     = 0,
        GS8     = 1,
        GS12    = 2,
        GS16    = 3,
    };

    enum class MagnetoSampleRate : uint8_t
    {
        Hz0_625     = 0,
        Hz1_25      = 1,
        Hz2_5       = 2,
        Hz5         = 3,
        Hz10        = 4,
        Hz20        = 5,
        Hz40        = 6,
        Hz80        = 7,
    };

    enum class MagnetoPerformance : uint8_t
    {
        LowPower                = 0,
        MediumPerformance       = 1,
        HighPerformance         = 2,
        UltraHighPerformance    = 3,

    };

    enum class MagnetoOperatingMode : uint8_t
    {
        ContinuousConversion    = 0,
        SingleConversion        = 1,
        PowerDown               = 2,
    };

    //----------------------------------------------------------------------------
    // AccelSettings
    //----------------------------------------------------------------------------
    struct GyroSettings
    {
        // Which axes are enabled
        bool enableX = true;
        bool enableY = true;
        bool enableZ = true;
        GyroScale scale = GyroScale::DPS245;
        GyroSampleRate sampleRate = GyroSampleRate::Hz952;
        uint8_t bandwidth = 0;
        bool lowPowerEnable = false;
        GyroHPF hpf = GyroHPF::Disabled;

        // Which axes are flipped
        bool flipX = false;
        bool flipY = false;
        bool flipZ = false;
        bool latchInterrupt = true;
    };

    //----------------------------------------------------------------------------
    // AccelSettings
    //----------------------------------------------------------------------------
    struct AccelSettings
    {
        // Which axes are enabled
        bool enableX = true;
        bool enableY = true;
        bool enableZ = true;
        AccelScale scale = AccelScale::G2;
        AccelSampleRate sampleRate = AccelSampleRate::Hz952;
        AccelBandwidth bandwidth = AccelBandwidth::Hz50;
        AccelHighResBandwidth highResBandwidth = AccelHighResBandwidth::ODR50;
    };

    //----------------------------------------------------------------------------
    // MagnetoSettings
    //----------------------------------------------------------------------------
    struct MagnetoSettings
    {
        MagnetoScale scale = MagnetoScale::GS4;
        MagnetoSampleRate sampleRate = MagnetoSampleRate::Hz80;
        bool tempCompensationEnable = false;
        MagnetoPerformance xyPerformance = MagnetoPerformance::UltraHighPerformance;
        MagnetoPerformance zPerformance = MagnetoPerformance::UltraHighPerformance;
        bool lowPowerEnable = false;
        MagnetoOperatingMode operatingMode = MagnetoOperatingMode::ContinuousConversion;
    };

    LM9DS1(const char *path = "/dev/i2c-1", int accelGyroSlaveAddress = 0x6B, int magnetoSlaveAddress = 0x1E);

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void init(const char *path = "/dev/i2c-1", int accelGyroSlaveAddress = 0x6B, int magnetoSlaveAddress = 0x1E);
    void initGyro(const GyroSettings &settings);
    void initAccel(const AccelSettings &settings);
    void initMagneto(const MagnetoSettings &settings);
    void calibrateAccelGyro();
    bool isAccelAvailable();
    bool isMagnetoAvailable(Axis axis = Axis::All);
    void readGyro(int16_t (&data)[3]);
    void readAccel(int16_t (&data)[3]);
    void readMagneto(int16_t (&data)[3]);
    void readGyro(float (&data)[3]);
    void readAccel(float (&data)[3]);
    void readMagneto(float (&data)[3]);

private:
    //----------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------
    static constexpr uint8_t AccelGyroID = 0x68;
    static constexpr uint8_t MagnetoID = 0x3D;

    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    enum class FIFOMode : uint8_t
    {
        Off                 = 0,
        Threshold           = 1,
        ContinuousTrigger   = 3,
        OffTrigger          = 4,
        Continuous          = 5,
    };

    enum class AccelGyroReg : uint8_t
    {
        ACT_THS = 0x04,
        ACT_DUR = 0x05,
        INT_GEN_CFG_XL = 0x06,
        INT_GEN_THS_X_XL = 0x07,
        INT_GEN_THS_Y_XL = 0x08,
        INT_GEN_THS_Z_XL = 0x09,
        INT_GEN_DUR_XL = 0x0A,
        REFERENCE_G = 0x0B,
        INT1_CTRL = 0x0C,
        INT2_CTRL = 0x0D,
        WHO_AM_I_XG = 0x0F,
        CTRL_REG1_G = 0x10,
        CTRL_REG2_G = 0x11,
        CTRL_REG3_G = 0x12,
        ORIENT_CFG_G = 0x13,
        INT_GEN_SRC_G = 0x14,
        OUT_TEMP_L = 0x15,
        OUT_TEMP_H = 0x16,
        STATUS_REG_0 = 0x17,
        OUT_X_L_G = 0x18,
        OUT_X_H_G = 0x19,
        OUT_Y_L_G = 0x1A,
        OUT_Y_H_G = 0x1B,
        OUT_Z_L_G = 0x1C,
        OUT_Z_H_G = 0x1D,
        CTRL_REG4 = 0x1E,
        CTRL_REG5_XL = 0x1F,
        CTRL_REG6_XL = 0x20,
        CTRL_REG7_XL = 0x21,
        CTRL_REG8 = 0x22,
        CTRL_REG9 = 0x23,
        CTRL_REG10 = 0x24,
        INT_GEN_SRC_XL = 0x26,
        STATUS_REG_1 = 0x27,
        OUT_X_L_XL = 0x28,
        OUT_X_H_XL = 0x29,
        OUT_Y_L_XL = 0x2A,
        OUT_Y_H_XL = 0x2B,
        OUT_Z_L_XL = 0x2C,
        OUT_Z_H_XL = 0x2D,
        FIFO_CTRL = 0x2E,
        FIFO_SRC = 0x2F,
        INT_GEN_CFG_G = 0x30,
        INT_GEN_THS_XH_G = 0x31,
        INT_GEN_THS_XL_G = 0x32,
        INT_GEN_THS_YH_G = 0x33,
        INT_GEN_THS_YL_G = 0x34,
        INT_GEN_THS_ZH_G = 0x35,
        INT_GEN_THS_ZL_G = 0x36,
        INT_GEN_DUR_G = 0x37,
    };

    enum class MagnetoReg : uint8_t
    {
        OFFSET_X_REG_L = 0x05,
        OFFSET_X_REG_H = 0x06,
        OFFSET_Y_REG_L = 0x07,
        OFFSET_Y_REG_H = 0x08,
        OFFSET_Z_REG_L = 0x09,
        OFFSET_Z_REG_H = 0x0A,
        WHO_AM_I = 0x0F,
        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,
        CTRL_REG5 = 0x24,
        STATUS_REG = 0x27,
        OUT_X_L = 0x28,
        OUT_X_H = 0x29,
        OUT_Y_L = 0x2A,
        OUT_Y_H = 0x2B,
        OUT_Z_L = 0x2C,
        OUT_Z_H = 0x2D,
        INT_CFG = 0x30,
        INT_SRC = 0x31,
        INT_THS_L = 0x32,
        INT_THS_H = 0x33,
    };

    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    uint8_t readByte(I2CInterface &interface, uint8_t address);

    template<typename T, size_t N>
    void readData(I2CInterface &interface, uint8_t address, T (&data)[N])
    {
        interface.writeByte(address | 0x80);
        interface.read(data);
    }

    void writeByte(I2CInterface &interface, uint8_t address, uint8_t byte);
    uint8_t readAccelGyroByte(AccelGyroReg reg);

    template<typename T, size_t N>
    void readMagnetoData(MagnetoReg reg, T (&data)[N])
    {
        readData(m_MagnetoI2C, static_cast<uint8_t>(reg), data);
    }

    template<typename T, size_t N>
    void readAccelGyroData(AccelGyroReg reg,  T (&data)[N])
    {
        readData(m_AccelGyroI2C, static_cast<uint8_t>(reg), data);
    }

    uint8_t readMagnetoByte(MagnetoReg reg);

    void writeAccelGyroByte(AccelGyroReg reg, uint8_t byte);
    void writeMagnetoByte(MagnetoReg reg, uint8_t byte);
    void setMagnetoOffset(MagnetoReg lowReg, MagnetoReg highReg, int16_t axisBias);
    void setFIFOEnabled(bool enabled);
    void setFIFOMode(FIFOMode mode, uint8_t threshold);
    unsigned int getNumFIFOSamples();

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    float m_MagnetoSensitivity;
    float m_AccelSensitivity;
    float m_GyroSensitivity;

    float m_MagnetoHardIronBias[3];
    float m_MagnetoSoftIronScale[3];

    int16_t m_AccelBias[3];
    int16_t m_GyroBias[3];

    I2CInterface m_AccelGyroI2C;
    I2CInterface m_MagnetoI2C;
};
} // BoBRobotics
#endif // __linux__
