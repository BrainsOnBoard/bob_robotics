#if defined(__linux__) && !defined(NO_I2C)

// BoB robotics includes
#include "common/lm9ds1_imu.h"
#include "common/logging.h"

// Standard C++ includes
#include <algorithm>
#include <stdexcept>

float
getGyroSensitivity(BoBRobotics::LM9DS1::GyroScale scale)
{
    switch (scale) {
    case BoBRobotics::LM9DS1::GyroScale::DPS245:
        return 0.00875f;
    case BoBRobotics::LM9DS1::GyroScale::DPS500:
        return 0.0175f;
    case BoBRobotics::LM9DS1::GyroScale::DPS2000:
        return 0.07f;
    default:
        throw std::runtime_error("Invalid parameter");
    }
}

float
getAccelSensitivity(BoBRobotics::LM9DS1::AccelScale scale)
{
    switch (scale) {
    case BoBRobotics::LM9DS1::AccelScale::G2:
        return 0.000061f;
    case BoBRobotics::LM9DS1::AccelScale::G4:
        return 0.000122f;
    case BoBRobotics::LM9DS1::AccelScale::G8:
        return 0.000244f;
    case BoBRobotics::LM9DS1::AccelScale::G16:
        return 0.000732f;
    default:
        throw std::runtime_error("Invalid parameter");
    }
}

float
getMagnetoSensitivity(BoBRobotics::LM9DS1::MagnetoScale scale)
{
    switch (scale) {
    case BoBRobotics::LM9DS1::MagnetoScale::GS4:
        return 0.00014f;
    case BoBRobotics::LM9DS1::MagnetoScale::GS8:
        return 0.00029f;
    case BoBRobotics::LM9DS1::MagnetoScale::GS12:
        return 0.00043f;
    case BoBRobotics::LM9DS1::MagnetoScale::GS16:
        return 0.00058f;
    default:
        throw std::runtime_error("Invalid parameter");
    }
}

namespace BoBRobotics {
//----------------------------------------------------------------------------
// Constants
//----------------------------------------------------------------------------
constexpr uint8_t LM9DS1::AccelGyroID;
constexpr uint8_t LM9DS1::MagnetoID;

LM9DS1::LM9DS1(const char *path, int accelGyroSlaveAddress, int magnetoSlaveAddress)
  : m_MagnetoSensitivity(1.0f)
  , m_AccelSensitivity(1.0f)
  , m_GyroSensitivity(1.0f)
  , m_MagnetoHardIronBias{ 0.20825f, -0.14784f, -1.60125f }
  , m_MagnetoSoftIronScale{ 0.960237096f, 1.426265591f, 0.795254653f }
  , m_AccelBias{ 0, 0, 0 }
  , m_GyroBias{ 0, 0, 0 }
{
    init(path, accelGyroSlaveAddress, magnetoSlaveAddress);
}

//----------------------------------------------------------------------------
// Public API
//----------------------------------------------------------------------------
void
LM9DS1::init(const char *path, int accelGyroSlaveAddress, int magnetoSlaveAddress)
{
    // Connect to I2C devices
    m_AccelGyroI2C.setup(path, accelGyroSlaveAddress);
    m_MagnetoI2C.setup(path, magnetoSlaveAddress);

    // Read identities
    const uint8_t accelGyroID = readAccelGyroByte(AccelGyroReg::WHO_AM_I_XG);
    const uint8_t magnetoID = readMagnetoByte(MagnetoReg::WHO_AM_I);

    // Check identities
    if (accelGyroID != AccelGyroID) {
        throw std::runtime_error("Accelerometer/gyro has wrong ID");
    }
    if (magnetoID != MagnetoID) {
        throw std::runtime_error("Magneto has wrong ID");
    }
}

void
LM9DS1::initGyro(const GyroSettings &settings)
{
    // Cache gyro sensitivity
    m_GyroSensitivity = getGyroSensitivity(settings.scale);

    // CTRL_REG1_G (Default value: 0x00)
    // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
    // ODR_G[2:0] - Output data rate selection
    // FS_G[1:0] - Gyroscope full-scale selection
    // BW_G[1:0] - Gyroscope bandwidth selection

    // To disable gyro, set sample rate bits to 0. We'll only set sample
    // rate if the gyro is enabled.
    uint8_t ctrlReg1Value = static_cast<uint8_t>(settings.sampleRate) << 5;
    ctrlReg1Value |= static_cast<uint8_t>(settings.scale);
    ctrlReg1Value |= (settings.bandwidth & 0x3);
    writeAccelGyroByte(AccelGyroReg::CTRL_REG1_G, ctrlReg1Value);

    // CTRL_REG2_G (Default value: 0x00)
    // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
    // INT_SEL[1:0] - INT selection configuration
    // OUT_SEL[1:0] - Out selection configuration
    writeAccelGyroByte(AccelGyroReg::CTRL_REG2_G, 0);

    // CTRL_REG3_G (Default value: 0x00)
    // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
    // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
    // HP_EN - HPF enable (0:disabled, 1: enabled)
    // HPCF_G[3:0] - HPF cutoff frequency
    uint8_t ctrlReg3Value = settings.lowPowerEnable ? (1 << 7) : 0;
    ctrlReg3Value |= static_cast<uint8_t>(settings.hpf);
    writeAccelGyroByte(AccelGyroReg::CTRL_REG3_G, ctrlReg3Value);

    // CTRL_REG4 (Default value: 0x38)
    // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
    // Zen_G - Z-axis output enable (0:disable, 1:enable)
    // Yen_G - Y-axis output enable (0:disable, 1:enable)
    // Xen_G - X-axis output enable (0:disable, 1:enable)
    // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
    // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
    uint8_t ctrlReg4Value = 0;
    if (settings.enableZ) {
        ctrlReg4Value |= (1 << 5);
    }
    if (settings.enableY) {
        ctrlReg4Value |= (1 << 4);
    }
    if (settings.enableX) {
        ctrlReg4Value |= (1 << 3);
    }
    if (settings.latchInterrupt) {
        ctrlReg4Value |= (1 << 1);
    }
    writeAccelGyroByte(AccelGyroReg::CTRL_REG4, ctrlReg4Value);

    // ORIENT_CFG_G (Default value: 0x00)
    // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
    // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
    // Orient [2:0] - Directional user orientation selection
    uint8_t orientCfgValue = 0;
    if (settings.flipX) {
        orientCfgValue |= (1 << 5);
    }
    if (settings.flipY) {
        orientCfgValue |= (1 << 4);
    }
    if (settings.flipZ) {
        orientCfgValue |= (1 << 3);
    }
    writeAccelGyroByte(AccelGyroReg::ORIENT_CFG_G, orientCfgValue);

    LOG_INFO << "Gyro initialised";
}

void
LM9DS1::initAccel(const AccelSettings &settings)
{
    // Cache accelerometer sensitivity
    m_AccelSensitivity = getAccelSensitivity(settings.scale);

    // CTRL_REG5_XL (0x1F) (Default value: 0x38)
    // [DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
    // DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
    //  00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
    // Zen_XL - Z-axis output enabled
    // Yen_XL - Y-axis output enabled
    // Xen_XL - X-axis output enabled
    uint8_t ctrlReg5Value = 0;
    if (settings.enableZ) {
        ctrlReg5Value |= (1 << 5);
    }
    if (settings.enableY) {
        ctrlReg5Value |= (1 << 4);
    }
    if (settings.enableX) {
        ctrlReg5Value |= (1 << 3);
    }

    writeAccelGyroByte(AccelGyroReg::CTRL_REG5_XL, ctrlReg5Value);

    // CTRL_REG6_XL (0x20) (Default value: 0x00)
    // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
    // ODR_XL[2:0] - Output data rate & power mode selection
    // FS_XL[1:0] - Full-scale selection
    // BW_SCAL_ODR - Bandwidth selection
    // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
    uint8_t ctrlReg6Value = static_cast<uint8_t>(settings.sampleRate) << 5;
    ctrlReg6Value |= static_cast<uint8_t>(settings.scale) << 3;
    ctrlReg6Value |= static_cast<uint8_t>(settings.bandwidth);
    writeAccelGyroByte(AccelGyroReg::CTRL_REG6_XL, ctrlReg6Value);

    // CTRL_REG7_XL (0x21) (Default value: 0x00)
    // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
    // HR - High resolution mode (0: disable, 1: enable)
    // DCF[1:0] - Digital filter cutoff frequency
    // FDS - Filtered data selection
    // HPIS1 - HPF enabled for interrupt function
    writeAccelGyroByte(AccelGyroReg::CTRL_REG7_XL, static_cast<uint8_t>(settings.highResBandwidth));

    LOG_INFO << "Accelerometer initialised";
}

void
LM9DS1::initMagneto(const MagnetoSettings &settings)
{
    // Cache magneto sensitivity
    m_MagnetoSensitivity = getMagnetoSensitivity(settings.scale);
    LOG_DEBUG << m_MagnetoSensitivity;
    // CTRL_REG1_M (Default value: 0x10)
    // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
    // TEMP_COMP - Temperature compensation
    // OM[1:0] - X & Y axes op mode selection
    // 00:low-power, 01:medium performance
    // 10: high performance, 11:ultra-high performance
    // DO[2:0] - Output data rate selection
    // ST - Self-test enable
    uint8_t ctrlReg1Value = 0;
    if (settings.tempCompensationEnable) {
        ctrlReg1Value |= (1 << 7);
    }
    ctrlReg1Value |= static_cast<uint8_t>(settings.xyPerformance) << 5;
    ctrlReg1Value |= static_cast<uint8_t>(settings.sampleRate) << 2;
    writeMagnetoByte(MagnetoReg::CTRL_REG1, ctrlReg1Value);

    // CTRL_REG2_M (Default value 0x00)
    // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
    // FS[1:0] - Full-scale configuration
    // REBOOT - Reboot memory content (0:normal, 1:reboot)
    // SOFT_RST - Reset config and user registers (0:default, 1:reset)
    const uint8_t ctrlReg2Value = static_cast<uint8_t>(settings.scale) << 5;
    writeMagnetoByte(MagnetoReg::CTRL_REG2, ctrlReg2Value);

    // CTRL_REG3_M (Default value: 0x03)
    // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
    // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
    // LP - Low-power mode cofiguration (1:enable)
    // SIM - SPI mode selection (0:write-only, 1:read/write enable)
    // MD[1:0] - Operating mode
    // 00:continuous conversion, 01:single-conversion,
    //  10,11: Power-down
    uint8_t ctrlReg3Value = 0;
    if (settings.lowPowerEnable) {
        ctrlReg3Value |= (1 << 5);
    }
    ctrlReg3Value |= static_cast<uint8_t>(settings.operatingMode);
    writeMagnetoByte(MagnetoReg::CTRL_REG3, ctrlReg3Value);

    // CTRL_REG4_M (Default value: 0x00)
    // [0][0][0][0][OMZ1][OMZ0][BLE][0]
    // OMZ[1:0] - Z-axis operative mode selection
    // 00:low-power mode, 01:medium performance
    // 10:high performance, 10:ultra-high performance
    // BLE - Big/little endian data
    const uint8_t ctrlReg4Value = static_cast<uint8_t>(settings.zPerformance) << 2;
    writeMagnetoByte(MagnetoReg::CTRL_REG4, ctrlReg4Value);

    // CTRL_REG5_M (Default value: 0x00)
    // [0][BDU][0][0][0][0][0][0]
    // BDU - Block data update for magnetic data
    // 0:continuous, 1:not updated until MSB/LSB are read
    writeMagnetoByte(MagnetoReg::CTRL_REG5, 0);

    LOG_INFO << "Magnetometer initialised";
}

void
LM9DS1::calibrateAccelGyro()
{
    LOG_INFO << "Calibrating accelerometer and gyroscope";

    setFIFOEnabled(true);
    setFIFOMode(FIFOMode::Threshold, 31);

    // Accumulate 32 samples
    unsigned int numSamples = 0;
    while (numSamples < 31) {
        numSamples = getNumFIFOSamples();
    }

    // Accumulate bias from sensor samples
    // **NOTE** 32-bit to prevent overflow
    int32_t accelBias[3] = { 0, 0, 0 };
    int32_t gyroBias[3] = { 0, 0, 0 };
    for (unsigned int i = 0; i < numSamples; i++) {
        // Read a sample from gyroscope
        int16_t gyroSample[3];
        readGyro(gyroSample);

        // Add to gyro bias
        gyroBias[0] += gyroSample[0];
        gyroBias[1] += gyroSample[1];
        gyroBias[2] += gyroSample[2];

        // Read a sample from accelerometer
        int16_t accelSample[3];
        readAccel(accelSample);

        // Add to acclerometer bias
        // **NOTE** we subtract gravity from Y as sensor is vertical in current robot
        accelBias[0] += accelSample[0];
        accelBias[1] += accelSample[1] - (int16_t)(1.0f / m_AccelSensitivity);
        accelBias[2] += accelSample[2];
    }

    //  Divide biases by number of samples to get means
    std::transform(std::begin(accelBias), std::end(accelBias), std::begin(m_AccelBias), [numSamples](int32_t v) { return v / (int32_t) numSamples; });
    std::transform(std::begin(gyroBias), std::end(gyroBias), std::begin(m_GyroBias), [numSamples](int32_t v) { return v / (int32_t) numSamples; });

    setFIFOEnabled(false);
    setFIFOMode(FIFOMode::Off, 0);

    LOG_DEBUG << "\tAccel bias:" << m_AccelBias[0] << "," << m_AccelBias[1] << "," << m_AccelBias[2];
    LOG_DEBUG << "\tGyro bias:" << m_GyroBias[0] << "," << m_GyroBias[1] << "," << m_GyroBias[2];
}

bool
LM9DS1::isAccelAvailable()
{
    return (readAccelGyroByte(AccelGyroReg::STATUS_REG_1) & (1 << 0));
}

bool
LM9DS1::isMagnetoAvailable(Axis axis)
{
    const uint8_t axisByte = static_cast<uint8_t>(axis);
    return ((readMagnetoByte(MagnetoReg::STATUS_REG) & (1 << axisByte)) >> axisByte);
}

void LM9DS1::readGyro(int16_t (&data)[3])
{
    readAccelGyroData(AccelGyroReg::OUT_X_L_G, data);
    std::transform(std::begin(data), std::end(data), std::begin(m_GyroBias), std::begin(data), [](int16_t v, int16_t bias) { return v - bias; });
}

void LM9DS1::readAccel(int16_t (&data)[3])
{
    readAccelGyroData(AccelGyroReg::OUT_X_L_XL, data);
    std::transform(std::begin(data), std::end(data), std::begin(m_AccelBias), std::begin(data), [](int16_t v, int16_t bias) { return v - bias; });
}

void LM9DS1::readMagneto(int16_t (&data)[3])
{
    readMagnetoData(MagnetoReg::OUT_X_L, data);
}

void
LM9DS1::readGyro(float (&data)[3])
{
    int16_t dataInt[3];
    readGyro(dataInt);
    std::transform(std::begin(dataInt), std::end(dataInt), std::begin(data), [this](int16_t v) { return m_GyroSensitivity * (float) v; });
}

void
LM9DS1::readAccel(float (&data)[3])
{
    int16_t dataInt[3];
    readAccel(dataInt);
    std::transform(std::begin(dataInt), std::end(dataInt), std::begin(data), [this](int16_t v) { return m_AccelSensitivity * (float) v; });
}

void
LM9DS1::readMagneto(float (&data)[3])
{
    int16_t dataInt[3];
    readMagneto(dataInt);
    for (unsigned int a = 0; a < 3; a++) {
        // Convert raw float values to gauss
        const float rawFloat = dataInt[a] * m_MagnetoSensitivity;

        // Apply hard and soft iron corrections
        data[a] = (rawFloat - m_MagnetoHardIronBias[a]) * m_MagnetoSoftIronScale[a];
    }
}

//----------------------------------------------------------------------------
// Private methods
//----------------------------------------------------------------------------
uint8_t
LM9DS1::readByte(I2CInterface &interface, uint8_t address)
{
    interface.writeByte(address);
    return interface.readByte();
}

void
LM9DS1::writeByte(I2CInterface &interface, uint8_t address, uint8_t byte)
{
    interface.writeByteCommand(address, byte);
}

uint8_t
LM9DS1::readAccelGyroByte(AccelGyroReg reg)
{
    return readByte(m_AccelGyroI2C, static_cast<uint8_t>(reg));
}

uint8_t
LM9DS1::readMagnetoByte(MagnetoReg reg)
{
    return readByte(m_MagnetoI2C, static_cast<uint8_t>(reg));
}

void
LM9DS1::writeAccelGyroByte(AccelGyroReg reg, uint8_t byte)
{
    writeByte(m_AccelGyroI2C, static_cast<uint8_t>(reg), byte);
}

void
LM9DS1::writeMagnetoByte(MagnetoReg reg, uint8_t byte)
{
    writeByte(m_MagnetoI2C, static_cast<uint8_t>(reg), byte);
}

void
LM9DS1::setMagnetoOffset(MagnetoReg lowReg, MagnetoReg highReg, int16_t axisBias)
{
    const uint8_t axisBiasMSB = (axisBias & 0xFF00) >> 8;
    const uint8_t axisBiasLSB = (axisBias & 0x00FF);
    writeMagnetoByte(lowReg, axisBiasLSB);
    writeMagnetoByte(highReg, axisBiasMSB);
}

void
LM9DS1::setFIFOEnabled(bool enabled)
{
    uint8_t ctrlReg9Value = readAccelGyroByte(AccelGyroReg::CTRL_REG9);
    if (enabled) {
        ctrlReg9Value |= (1 << 1);
    } else {
        ctrlReg9Value &= ~(1 << 1);
    }
    writeAccelGyroByte(AccelGyroReg::CTRL_REG9, ctrlReg9Value);
}

void
LM9DS1::setFIFOMode(FIFOMode mode, uint8_t threshold)
{
    // Clamp threshold
    threshold = std::min((uint8_t) 31, threshold);

    const uint8_t fifoCtrl = (static_cast<uint8_t>(mode) << 5) | threshold;
    writeAccelGyroByte(AccelGyroReg::FIFO_CTRL, fifoCtrl);
}

unsigned int
LM9DS1::getNumFIFOSamples()
{
    return readAccelGyroByte(AccelGyroReg::FIFO_SRC) & 0x3F;
}
} // BoBRobotics
#endif // __linux__ && !NO_I2C
