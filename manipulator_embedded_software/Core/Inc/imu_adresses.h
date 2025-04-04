#ifndef SENSOR_REGISTERS_H
#define SENSOR_REGISTERS_H

/** I2C1 GPIO Configuration
    PB8/D15    ------> I2C1_SCL
    PB9/D14    ------> I2C1_SDA   

    ICM-20948 detected successfully!
    System Initialized.
    Device ID: 0xEA

    CHIP: ICM-20948
*/

// CTRL + SHIFT + I     <--- code cleanup

// Device ID
#define DEVICE_ID        0xEA
#define DEVICE_ADRESS    0x69

// Accelerometer Output Registers
#define ACCEL_XOUT_H    0x2D  // Register 45
#define ACCEL_XOUT_L    0x2E  // Register 46
#define ACCEL_YOUT_H    0x2F  // Register 47
#define ACCEL_YOUT_L    0x30  // Register 48
#define ACCEL_ZOUT_H    0x31  // Register 49
#define ACCEL_ZOUT_L    0x32  // Register 50

// Gyroscope Output Registers
#define GYRO_XOUT_H     0x33  // Register 51
#define GYRO_XOUT_L     0x34  // Register 52
#define GYRO_YOUT_H     0x35  // Register 53
#define GYRO_YOUT_L     0x36  // Register 54
#define GYRO_ZOUT_H     0x37  // Register 55
#define GYRO_ZOUT_L     0x38  // Register 56

// Temperature Output Registers
#define TEMP_OUT_H      0x39  // Register 57
#define TEMP_OUT_L      0x3A  // Register 58

// Magnetometer Output Registers (AK09916, slave device)
#define MAG_CTRL_REG1   0x0A  // Magnetometer control register
#define MAG_XOUT_L      0x11  // Magnetometer X-axis Low Byte
#define MAG_XOUT_H      0x12  // Magnetometer X-axis High Byte
#define MAG_YOUT_L      0x13  // Magnetometer Y-axis Low Byte
#define MAG_YOUT_H      0x14  // Magnetometer Y-axis High Byte
#define MAG_ZOUT_L      0x15  // Magnetometer Z-axis Low Byte
#define MAG_ZOUT_H      0x16  // Magnetometer Z-axis High Byte

// External Sensor Data Registers
#define EXT_SLV_SENS_DATA_00  0x3B  // Register 59
#define EXT_SLV_SENS_DATA_01  0x3C  // Register 60
#define EXT_SLV_SENS_DATA_02  0x3D  // Register 61
#define EXT_SLV_SENS_DATA_03  0x3E  // Register 62
#define EXT_SLV_SENS_DATA_04  0x3F  // Register 63
#define EXT_SLV_SENS_DATA_05  0x40  // Register 64
#define EXT_SLV_SENS_DATA_06  0x41  // Register 65
#define EXT_SLV_SENS_DATA_07  0x42  // Register 66
#define EXT_SLV_SENS_DATA_08  0x43  // Register 67
#define EXT_SLV_SENS_DATA_09  0x44  // Register 68
#define EXT_SLV_SENS_DATA_10  0x45  // Register 69
#define EXT_SLV_SENS_DATA_11  0x46  // Register 70
#define EXT_SLV_SENS_DATA_12  0x47  // Register 71
#define EXT_SLV_SENS_DATA_13  0x48  // Register 72
#define EXT_SLV_SENS_DATA_14  0x49  // Register 73
#define EXT_SLV_SENS_DATA_15  0x4A  // Register 74
#define EXT_SLV_SENS_DATA_16  0x4B  // Register 75
#define EXT_SLV_SENS_DATA_17  0x4C  // Register 76
#define EXT_SLV_SENS_DATA_18  0x4D  // Register 77
#define EXT_SLV_SENS_DATA_19  0x4E  // Register 78
#define EXT_SLV_SENS_DATA_20  0x4F  // Register 79
#define EXT_SLV_SENS_DATA_21  0x50  // Register 80
#define EXT_SLV_SENS_DATA_22  0x51  // Register 81
#define EXT_SLV_SENS_DATA_23  0x52  // Register 82

// WHO_AM_I Register
#define WHO_AM_I        0x00  // Register to read chip ID (should be 0xEA for ICM-20948)

// Power Management Registers
#define PWR_MGMT_1      0x06  // Power Management 1
#define PWR_MGMT_2      0x07  // Power Management 2

// User Control Register
#define USER_CTRL       0x03  // User Control Register

// FIFO Configuration Registers
#define FIFO_EN_1       0x66  // FIFO Enable 1
#define FIFO_EN_2       0x67  // FIFO Enable 2
#define FIFO_RST        0x68  // FIFO Reset
#define FIFO_MODE       0x69  // FIFO Mode
#define FIFO_COUNTH     0x70  // FIFO Count High Byte
#define FIFO_COUNTL     0x71  // FIFO Count Low Byte
#define FIFO_R_W        0x72  // FIFO Read/Write
#define FIFO_CFG        0x76  // FIFO Configuration

// Interrupt Configuration Registers
#define INT_PIN_CFG     0x0F  // Interrupt Pin Configuration
#define INT_ENABLE      0x10  // Interrupt Enable
#define INT_STATUS      0x19  // Interrupt Status

// Register Bank Selection
#define REG_BANK_SEL    0x7F  // Register Bank Select

#endif // SENSOR_REGISTERS_H