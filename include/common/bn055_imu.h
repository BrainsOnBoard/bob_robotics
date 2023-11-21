#pragma once
#ifdef __linux__

// Standard C++ includes
#include <array>

// Standard C includes
#include <cstdint>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// BoB robotics third party includes
#include "third_party/units.h"

// BoB robotics includes
#include "i2c_interface.h"

//----------------------------------------------------------------------------
// BoBRobotics::BN055
//----------------------------------------------------------------------------
namespace BoBRobotics {
class BN055
{
private:
    //----------------------------------------------------------------------------
    // Units
    //----------------------------------------------------------------------------
    using degree_t = units::angle::degree_t;
    using degrees_per_second_t = units::angular_velocity::degrees_per_second_t;
    
#if !defined(DISABLE_PREDEFINED_UNITS) || defined(ENABLE_PREDEFINED_ACCELERATION_UNITS)
    using meters_per_second_squared_t = units::acceleration::meters_per_second_squared_t;
#endif

#if !defined(DISABLE_PREDEFINED_UNITS) || defined(ENABLE_PREDEFINED_MAGNETIC_FIELD_STRENGTH_UNITS)
    using microtesla_t = units::magnetic_field_strength::microtesla_t;
#endif
    
    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    // Registers
    enum class Register : uint8_t
    {
        // Page id register definition
        PAGE_ID_ADDR = 0X07,

        // PAGE0 REGISTER DEFINITION START
        CHIP_ID_ADDR = 0x00,
        ACCEL_REV_ID_ADDR = 0x01,
        MAG_REV_ID_ADDR = 0x02,
        GYRO_REV_ID_ADDR = 0x03,
        SW_REV_ID_LSB_ADDR = 0x04,
        SW_REV_ID_MSB_ADDR = 0x05,
        BL_REV_ID_ADDR = 0X06,

        // Accel data register
        ACCEL_DATA_X_LSB_ADDR = 0X08,
        ACCEL_DATA_X_MSB_ADDR = 0X09,
        ACCEL_DATA_Y_LSB_ADDR = 0X0A,
        ACCEL_DATA_Y_MSB_ADDR = 0X0B,
        ACCEL_DATA_Z_LSB_ADDR = 0X0C,
        ACCEL_DATA_Z_MSB_ADDR = 0X0D,

        // Mag data register
        MAG_DATA_X_LSB_ADDR = 0X0E,
        MAG_DATA_X_MSB_ADDR = 0X0F,
        MAG_DATA_Y_LSB_ADDR = 0X10,
        MAG_DATA_Y_MSB_ADDR = 0X11,
        MAG_DATA_Z_LSB_ADDR = 0X12,
        MAG_DATA_Z_MSB_ADDR = 0X13,

        // Gyro data registers
        GYRO_DATA_X_LSB_ADDR = 0X14,
        GYRO_DATA_X_MSB_ADDR = 0X15,
        GYRO_DATA_Y_LSB_ADDR = 0X16,
        GYRO_DATA_Y_MSB_ADDR = 0X17,
        GYRO_DATA_Z_LSB_ADDR = 0X18,
        GYRO_DATA_Z_MSB_ADDR = 0X19,

        // Euler data registers
        EULER_H_LSB_ADDR = 0X1A,
        EULER_H_MSB_ADDR = 0X1B,
        EULER_R_LSB_ADDR = 0X1C,
        EULER_R_MSB_ADDR = 0X1D,
        EULER_P_LSB_ADDR = 0X1E,
        EULER_P_MSB_ADDR = 0X1F,

        // Quaternion data registers
        QUATERNION_DATA_W_LSB_ADDR = 0X20,
        QUATERNION_DATA_W_MSB_ADDR = 0X21,
        QUATERNION_DATA_X_LSB_ADDR = 0X22,
        QUATERNION_DATA_X_MSB_ADDR = 0X23,
        QUATERNION_DATA_Y_LSB_ADDR = 0X24,
        QUATERNION_DATA_Y_MSB_ADDR = 0X25,
        QUATERNION_DATA_Z_LSB_ADDR = 0X26,
        QUATERNION_DATA_Z_MSB_ADDR = 0X27,

        // Linear acceleration data registers
        LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,
        LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
        LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
        LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
        LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
        LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

        // Gravity data registers
        GRAVITY_DATA_X_LSB_ADDR = 0X2E,
        GRAVITY_DATA_X_MSB_ADDR = 0X2F,
        GRAVITY_DATA_Y_LSB_ADDR = 0X30,
        GRAVITY_DATA_Y_MSB_ADDR = 0X31,
        GRAVITY_DATA_Z_LSB_ADDR = 0X32,
        GRAVITY_DATA_Z_MSB_ADDR = 0X33,

        // Temperature data register
        TEMP_ADDR = 0X34,

        // Status registers
        CALIB_STAT_ADDR = 0X35,
        SELFTEST_RESULT_ADDR = 0X36,
        INTR_STAT_ADDR = 0X37,

        SYS_CLK_STAT_ADDR = 0X38,
        SYS_STAT_ADDR = 0X39,
        SYS_ERR_ADDR = 0X3A,

        // Unit selection register
        UNIT_SEL_ADDR = 0X3B,
        DATA_SELECT_ADDR = 0X3C,

        // Mode registers
        OPR_MODE_ADDR = 0X3D,
        PWR_MODE_ADDR = 0X3E,

        SYS_TRIGGER_ADDR = 0X3F,
        TEMP_SOURCE_ADDR = 0X40,

        // Axis remap registers
        AXIS_MAP_CONFIG_ADDR = 0X41,
        AXIS_MAP_SIGN_ADDR = 0X42,

        // SIC registers 
        SIC_MATRIX_0_LSB_ADDR = 0X43,
        SIC_MATRIX_0_MSB_ADDR = 0X44,
        SIC_MATRIX_1_LSB_ADDR = 0X45,
        SIC_MATRIX_1_MSB_ADDR = 0X46,
        SIC_MATRIX_2_LSB_ADDR = 0X47,
        SIC_MATRIX_2_MSB_ADDR = 0X48,
        SIC_MATRIX_3_LSB_ADDR = 0X49,
        SIC_MATRIX_3_MSB_ADDR = 0X4A,
        SIC_MATRIX_4_LSB_ADDR = 0X4B,
        SIC_MATRIX_4_MSB_ADDR = 0X4C,
        SIC_MATRIX_5_LSB_ADDR = 0X4D,
        SIC_MATRIX_5_MSB_ADDR = 0X4E,
        SIC_MATRIX_6_LSB_ADDR = 0X4F,
        SIC_MATRIX_6_MSB_ADDR = 0X50,
        SIC_MATRIX_7_LSB_ADDR = 0X51,
        SIC_MATRIX_7_MSB_ADDR = 0X52,
        SIC_MATRIX_8_LSB_ADDR = 0X53,
        SIC_MATRIX_8_MSB_ADDR = 0X54,

        // Accelerometer Offset registers
        ACCEL_OFFSET_X_LSB_ADDR = 0X55,
        ACCEL_OFFSET_X_MSB_ADDR = 0X56,
        ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
        ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
        ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
        ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

        // Magnetometer Offset registers 
        MAG_OFFSET_X_LSB_ADDR = 0X5B,
        MAG_OFFSET_X_MSB_ADDR = 0X5C,
        MAG_OFFSET_Y_LSB_ADDR = 0X5D,
        MAG_OFFSET_Y_MSB_ADDR = 0X5E,
        MAG_OFFSET_Z_LSB_ADDR = 0X5F,
        MAG_OFFSET_Z_MSB_ADDR = 0X60,

        // Gyroscope Offset register s
        GYRO_OFFSET_X_LSB_ADDR = 0X61,
        GYRO_OFFSET_X_MSB_ADDR = 0X62,
        GYRO_OFFSET_Y_LSB_ADDR = 0X63,
        GYRO_OFFSET_Y_MSB_ADDR = 0X64,
        GYRO_OFFSET_Z_LSB_ADDR = 0X65,
        GYRO_OFFSET_Z_MSB_ADDR = 0X66,

        // Radius registers
        ACCEL_RADIUS_LSB_ADDR = 0X67,
        ACCEL_RADIUS_MSB_ADDR = 0X68,
        MAG_RADIUS_LSB_ADDR = 0X69,
        MAG_RADIUS_MSB_ADDR = 0X6A
    };

    // BNO055 power setting
    enum class PowerMode : uint8_t
    {
        NORMAL = 0X00,
        LOWPOWER = 0X01,
        SUSPEND = 0X02
    };

    // Remap settings
    enum class RemapConfig : uint8_t
    {
        P0 = 0x21,
        P1 = 0x24, // default
        P2 = 0x24,
        P3 = 0x21,
        P4 = 0x24,
        P5 = 0x21,
        P6 = 0x21,
        P7 = 0x24
    };

    // Remap Signs
    enum class RemapSign : uint8_t
    {
        REMAP_SIGN_P0 = 0x04,
        REMAP_SIGN_P1 = 0x00, // default
        REMAP_SIGN_P2 = 0x06,
        REMAP_SIGN_P3 = 0x02,
        REMAP_SIGN_P4 = 0x03,
        REMAP_SIGN_P5 = 0x01,
        REMAP_SIGN_P6 = 0x07,
        REMAP_SIGN_P7 = 0x05
    };

public:
    //----------------------------------------------------------------------------
    // Enumerations
    //----------------------------------------------------------------------------
    // Operation mode settings
    enum class OperationMode : uint8_t
    {
        CONFIG = 0X00,
        ACCONLY = 0X01,
        MAGONLY = 0X02,
        GYRONLY = 0X03,
        ACCMAG = 0X04,
        ACCGYRO = 0X05,
        MAGGYRO = 0X06,
        AMG = 0X07,
        IMUPLUS = 0X08,
        COMPASS = 0X09,
        M4G = 0X0A,
        NDOF_FMC_OFF = 0X0B,
        NDOF = 0X0C
    };

    enum class VectorType : uint8_t
    {
        ACCELEROMETER   = static_cast<uint8_t>(Register::ACCEL_DATA_X_LSB_ADDR),
        MAGNETOMETER    = static_cast<uint8_t>(Register::MAG_DATA_X_LSB_ADDR),
        GYROSCOPE       = static_cast<uint8_t>(Register::GYRO_DATA_X_LSB_ADDR),
        EULER           = static_cast<uint8_t>(Register::EULER_H_LSB_ADDR),
        LINEARACCEL     = static_cast<uint8_t>(Register::LINEAR_ACCEL_DATA_X_LSB_ADDR),
        GRAVITY         = static_cast<uint8_t>(Register::GRAVITY_DATA_X_LSB_ADDR)
    };

    BN055(OperationMode mode = OperationMode::NDOF, const char *path = I2C_DEVICE_DEFAULT, int slaveAddress = 0x28);

    //----------------------------------------------------------------------------
    // Public API
    //----------------------------------------------------------------------------
    void setup(OperationMode mode = OperationMode::NDOF);
    
    Eigen::Quaternionf getQuaternion();
    
    Eigen::Vector3f getVector(VectorType vectorType = VectorType::EULER);
    
    std::array<degree_t, 3> getEulerAngles()
    {
        return getUnitVector<units::angle::degree_t>(VectorType::EULER);
    }
    
    std::array<degrees_per_second_t, 3> getGyroscope()
    {
        return getUnitVector<degrees_per_second_t>(VectorType::GYROSCOPE);
    }
    
#if !defined(DISABLE_PREDEFINED_UNITS) || defined(ENABLE_PREDEFINED_ACCELERATION_UNITS)
    std::array<meters_per_second_squared_t, 3> getAccelerometer()
    {
        return getUnitVector<meters_per_second_squared_t>(VectorType::ACCELEROMETER);
    }
    
    std::array<meters_per_second_squared_t, 3> getLinearAccel()
    {
        return getUnitVector<meters_per_second_squared_t>(VectorType::LINEARACCEL);
    }
    
    std::array<meters_per_second_squared_t, 3> getGravity()
    {
        return getUnitVector<meters_per_second_squared_t>(VectorType::GRAVITY);
    }
#endif
    
#if !defined(DISABLE_PREDEFINED_UNITS) || defined(ENABLE_PREDEFINED_MAGNETIC_FIELD_STRENGTH_UNITS)
    std::array<microtesla_t, 3> getMagnetometer()
    {
        return getUnitVector<microtesla_t>(VectorType::MAGNETOMETER);
    }
#endif

private:
    //----------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------
    static constexpr uint8_t imuID = 0xA0;

    //----------------------------------------------------------------------------
    // Private methods
    //----------------------------------------------------------------------------
    template<typename T>
    std::array<T, 3> getUnitVector(VectorType vectorType)
    {
        const Eigen::Vector3f vector = getVector(vectorType);
        return {T{vector[0]}, T{vector[1]}, T{vector[2]}};
    }
    
    uint8_t readByte(uint8_t address);
    void writeByte(uint8_t address, uint8_t data);

    template<typename T, size_t N>
    void readData(uint8_t address, T (&data)[N])
    {
        m_IMU.writeByte(address);
        m_IMU.read(data);
    }

    uint8_t readRegister(Register reg);
    void writeRegister(Register reg, uint8_t data);

    OperationMode setMode(OperationMode mode);

    //----------------------------------------------------------------------------
    // Members
    //----------------------------------------------------------------------------
    I2CInterface m_IMU;
    OperationMode m_Mode;
};
}   // namespace BoBRobotics

#endif  // __linux__
