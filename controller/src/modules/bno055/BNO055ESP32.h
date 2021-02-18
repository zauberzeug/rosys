// MIT License

// Copyright (c) 2019 ShellAddicted <github.com/ShellAddicted>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*
        https://www.bosch-sensortec.com/bst/products/all_products/bno055
        Reference Datasheet: BST_BNO055_DS000_14 (consulted in January 2018)
*/

#ifndef _BNO055ESP32_H_
#define _BNO055ESP32_H_

#define BNO055_DEBUG_OFF // uncomment this to DISABLE DEBUG LOGS

#include <cstring> //memset, memcpy
#include <exception>
#include <string>
#include "driver/i2c.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ACK_EN 0x01

#ifndef BNO055_DEBUG_OFF
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"

#define DEFAULT_UART_TIMEOUT_MS 30 // you can try to decrease/increase this. (DEFAULT: 30)

// BNO055 Registers(Table 4-1, Pag 51)
typedef enum
{
    // PAGE 1
    BNO055_REG_ACC_CONFIG = 0x08,
    BN0055_REG_MAG_CONFIG = 0x09,
    BNO055_REG_GYR_CONFIG_0 = 0x0A,
    BNO055_REG_GYR_CONFIG_1 = 0x0B,

    BNO055_REG_ACC_SLEEP_CONFIG = 0x0C,
    BNO055_REG_GYR_SLEEP_CONFIG = 0x0D,

    BNO055_REG_INT_MSK = 0x0F,
    BNO055_REG_INT_EN = 0x10,

    BNO055_REG_ACC_AM_THRES = 0x11,
    BNO055_REG_ACC_INT_SETTINGS = 0x12,
    BNO055_REG_ACC_HG_DURATION = 0x13,
    BNO055_REG_ACC_HG_THRES = 0x14,
    BNO055_REG_ACC_NM_THRES = 0x15,
    BNO055_REG_ACC_NM_SET = 0x16,
    BNO055_REG_GYR_INT_SETTING = 0x17,
    BNO055_REG_GYR_HR_X_SET = 0x18,
    BNO055_REG_GYR_DUR_X = 0x19,
    BNO055_REG_GYR_HR_Y_SET = 0x1A,
    BNO055_REG_GYR_DUR_Y = 0x1B,
    BNO055_REG_GYR_HR_Z_SET = 0x1C,
    BNO055_REG_GYR_DUR_Z = 0x1D,
    BNO055_REG_GYR_AM_THRES = 0x1E,
    BNO055_REG_GYR_AM_SET = 0x1F,

    BNO055_REG_PAGE_ID = 0x07,

    // PAGE 0
    BNO055_REG_CHIP_ID = 0x00,
    BNO055_REG_ACC_ID = 0x01,
    BNO055_REG_MAG_ID = 0x02,
    BNO055_REG_GYRO_ID = 0x03,
    BNO055_REG_SW_REV_ID_LSB = 0x04,
    BNO055_REG_SW_REV_ID_MSB = 0x05,
    BNO055_REG_BL_REV_ID = 0x06,

    BNO055_REG_ACC_DATA_X_LSB = 0x08,
    BNO055_REG_ACC_DATA_X_MSB = 0x09,
    BNO055_REG_ACC_DATA_Y_LSB = 0x0A,
    BNO055_REG_ACC_DATA_Y_MSB = 0x0B,
    BNO055_REG_ACC_DATA_Z_LSB = 0x0C,
    BNO055_REG_ACC_DATA_Z_MSB = 0x0D,

    BNO055_REG_MAG_DATA_X_LSB = 0x0E,
    BNO055_REG_MAG_DATA_X_MSB = 0x0F,
    BNO055_REG_MAG_DATA_Y_LSB = 0x10,
    BNO055_REG_MAG_DATA_Y_MSB = 0x11,
    BNO055_REG_MAG_DATA_Z_LSB = 0x12,
    BNO055_REG_MAG_DATA_Z_MSB = 0x13,

    BNO055_REG_GYR_DATA_X_LSB = 0x14,
    BNO055_REG_GYR_DATA_X_MSB = 0x15,
    BNO055_REG_GYR_DATA_Y_LSB = 0x16,
    BNO055_REG_GYR_DATA_Y_MSB = 0x17,
    BNO055_REG_GYR_DATA_Z_LSB = 0x18,
    BNO055_REG_GYR_DATA_Z_MSB = 0x19,

    BNO055_REG_EUL_HEADING_LSB = 0x1A,
    BNO055_REG_EUL_HEADING_MSB = 0x1B,
    BNO055_REG_EUL_ROLL_LSB = 0x1C,
    BNO055_REG_EUL_ROLL_MSB = 0x1D,
    BNO055_REG_EUL_PITCH_LSB = 0x1E,
    BNO055_REG_EUL_PITCH_MSB = 0x1F,

    BNO055_REG_QUA_DATA_W_LSB = 0x20,
    BNO055_REG_QUA_DATA_W_MSB = 0x21,
    BNO055_REG_QUA_DATA_X_LSB = 0x22,
    BNO055_REG_QUA_DATA_X_MSB = 0x23,
    BNO055_REG_QUA_DATA_Y_LSB = 0x24,
    BNO055_REG_QUA_DATA_Y_MSB = 0x25,
    BNO055_REG_QUA_DATA_Z_LSB = 0x26,
    BNO055_REG_QUA_DATA_Z_MSB = 0x27,

    BNO055_REG_LIA_DATA_X_LSB = 0x28,
    BNO055_REG_LIA_DATA_X_MSB = 0x29,
    BNO055_REG_LIA_DATA_Y_LSB = 0x2A,
    BNO055_REG_LIA_DATA_Y_MSB = 0x2B,
    BNO055_REG_LIA_DATA_Z_LSB = 0x2C,
    BNO055_REG_LIA_DATA_Z_MSB = 0x2D,

    BNO055_REG_GRV_DATA_X_LSB = 0x2E,
    BNO055_REG_GRV_DATA_X_MSB = 0x2F,
    BNO055_REG_GRV_DATA_Y_LSB = 0x30,
    BNO055_REG_GRV_DATA_Y_MSB = 0x31,
    BNO055_REG_GRV_DATA_Z_LSB = 0x32,
    BNO055_REG_GRV_DATA_Z_MSB = 0x33,

    BNO055_REG_TEMP = 0x34,

    BNO055_REG_CALIB_STAT = 0x35,
    BNO055_REG_ST_RESULT = 0x36,
    BNO055_REG_INT_STA = 0x37,

    BNO055_REG_SYS_CLK_STAT = 0x38,
    BNO055_REG_SYS_STATUS = 0x39,
    BNO055_REG_SYS_ERR = 0x3A,

    BNO055_REG_UNIT_SEL = 0x3B,

    BNO055_REG_OPR_MODE = 0x3D,
    BNO055_REG_PWR_MODE = 0x3E,
    BNO055_REG_SYS_TRIGGER = 0x3F,
    BNO055_REG_TEMP_SOURCE = 0x40,

    BNO055_REG_AXIS_MAP_CONFIG = 0x41,
    BNO055_REG_AXIS_MAP_SIGN = 0x42,

    BNO055_REG_ACC_OFFSET_X_LSB = 0x55,
    BNO055_REG_ACC_OFFSET_X_MSB = 0x56,
    BNO055_REG_ACC_OFFSET_Y_LSB = 0x57,
    BNO055_REG_ACC_OFFSET_Y_MSB = 0x58,
    BNO055_REG_ACC_OFFSET_Z_LSB = 0x59,
    BNO055_REG_ACC_OFFSET_Z_MSB = 0x5A,

    BNO055_REG_MAG_OFFSET_X_LSB = 0x5B,
    BNO055_REG_MAG_OFFSET_X_MSB = 0x5C,
    BNO055_REG_MAG_OFFSET_Y_LSB = 0x5D,
    BNO055_REG_MAG_OFFSET_Y_MSB = 0x5E,
    BNO055_REG_MAG_OFFSET_Z_LSB = 0x5F,
    BNO055_REG_MAG_OFFSET_Z_MSB = 0x60,

    BNO055_REG_GYR_OFFSET_X_LSB = 0x61,
    BNO055_REG_GYR_OFFSET_X_MSB = 0x62,
    BNO055_REG_GYR_OFFSET_Y_LSB = 0x63,
    BNO055_REG_GYR_OFFSET_Y_MSB = 0x64,
    BNO055_REG_GYR_OFFSET_Z_LSB = 0x65,
    BNO055_REG_GYR_OFFSET_Z_MSB = 0x66,

    BNO055_REG_ACC_RADIUS_LSB = 0x67,
    BNO055_REG_ACC_RADIUS_MSB = 0x68,

    BNO055_REG_MAG_RADIUS_LSB = 0x69,
    BNO055_REG_MAG_RADIUS_MSB = 0x6A
} bno055_reg_t;

/* System Status [SYS_STATUS] (sec: 4.3.58)
        0 = Idle
        1 = System Error
        2 = Initializing Peripherals
        3 = System Iniitalization
        4 = Executing Self-Test
        5 = Sensor fusion algorithm running
        6 = System running without fusion algorithms
*/
typedef enum
{
    BNO055_SYSTEM_STATUS_IDLE = 0x00,
    BNO055_SYSTEM_STATUS_SYSTEM_ERROR = 0x01,
    BNO055_SYSTEM_STATUS_INITIALIZING_PERIPHERALS = 0x02,
    BNO055_SYSTEM_STATUS_SYSTEM_INITIALIZATION = 0x03,
    BNO055_SYSTEM_STATUS_EXECUTING_SELF_TEST = 0x04,
    BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
    BNO055_SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING = 0x06
} bno055_system_status_t;

/* System Error [SYS_ERR] (sec: 4.3.59)
        0 = No error
        1 = Peripheral initialization error
        2 = System initialization error
        3 = Self test result failed
        4 = Register map value out of range
        5 = Register map address out of range
        6 = Register map write error
        7 = BNO low power mode not available for selected operat ion mode
        8 = Accelerometer power mode not available
        9 = Fusion algorithm configuration error
        A = Sensor configuration error
*/
typedef enum
{
    BNO055_SYSTEM_ERROR_NO_ERROR = 0x00,
    BNO055_SYSTEM_ERROR_PERIPHERAL_INITIALIZATION_ERROR = 0x01,
    BNO055_SYSTEM_ERROR_SYSTEM_INITIALIZATION_ERROR = 0x02,
    BNO055_SYSTEM_ERROR_SELF_TEST_FAILED = 0x03,
    BNO055_SYSTEM_ERROR_REG_MAP_VAL_OUT_OF_RANGE = 0x04,
    BNO055_SYSTEM_ERROR_REG_MAP_ADDR_OUT_OF_RANGE = 0x05,
    BNO055_SYSTEM_ERROR_REG_MAP_WRITE_ERROR = 0x06,
    BNO055_SYSTEM_ERROR_LOW_PWR_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE = 0x07,
    BNO055_SYSTEM_ERROR_ACCEL_PWR_MODE_NOT_AVAILABLE = 0x08,
    BNO055_SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR = 0x09,
    BNO055_SYSTEM_ERROR_SENSOR_CONF_ERROR = 0x0A
} bno055_system_error_t;

typedef enum
{
    BNO055_PWR_MODE_NORMAL = 0x00,
    BNO055_PWR_MODE_LOWPOWER = 0x01,
    BNO055_PWR_MODE_SUSPEND = 0x02
} bno055_powermode_t;

typedef enum
{
    BNO055_REMAP_CONFIG_P0 = 0x21,
    BNO055_REMAP_CONFIG_P1 = 0x24,
    BNO055_REMAP_CONFIG_P2 = 0x24,
    BNO055_REMAP_CONFIG_P3 = 0x21,
    BNO055_REMAP_CONFIG_P4 = 0x24,
    BNO055_REMAP_CONFIG_P5 = 0x21,
    BNO055_REMAP_CONFIG_P6 = 0x21,
    BNO055_REMAP_CONFIG_P7 = 0x24
} bno055_axis_config_t;

typedef enum
{
    BNO055_REMAP_SIGN_P0 = 0x04,
    BNO055_REMAP_SIGN_P1 = 0x00,
    BNO055_REMAP_SIGN_P2 = 0x06,
    BNO055_REMAP_SIGN_P3 = 0x02,
    BNO055_REMAP_SIGN_P4 = 0x03,
    BNO055_REMAP_SIGN_P5 = 0x01,
    BNO055_REMAP_SIGN_P6 = 0x07,
    BNO055_REMAP_SIGN_P7 = 0x05
} bno055_axis_sign_t;

typedef struct
{
    uint8_t mcuState = 0;
    uint8_t gyrState = 0;
    uint8_t magState = 0;
    uint8_t accState = 0;
} bno055_self_test_result_t;

typedef struct
{
    double x = 0;
    double y = 0;
    double z = 0;
} bno055_vector_t;

typedef struct
{
    double w = 0;
    double x = 0;
    double y = 0;
    double z = 0;
} bno055_quaternion_t;

typedef enum
{
    BNO055_UNIT_ACCEL_MS2 = 0x00, // m/s²
    BNO055_UNIT_ACCEL_MG = 0X01
} bno055_accel_unit_t;

typedef enum
{
    BNO055_UNIT_ANGULAR_RATE_DPS = 0x00,
    BNO055_UNIT_ANGULAR_RATE_RPS = 0x02
} bno055_angular_rate_unit_t;

typedef enum
{
    BNO055_UNIT_EULER_DEGREES = 0x00,
    BNO055_UNIT_EULER_RADIANS = 0x04
} bno055_euler_unit_t;

typedef enum
{
    BNO055_UNIT_TEMP_C = 0x00,
    BNO055_UNIT_TEMP_F = 0x10
} bno055_temperature_unit_t;

typedef enum
{
    BNO055_DATA_FORMAT_WINDOWS = 0x00,
    BNO055_DATA_FORMAT_ANDROID = 0x80
} bno055_data_output_format_t;

typedef enum
{
    BNO055_CONF_ACCEL_RANGE_2G = 0x00,
    BNO055_CONF_ACCEL_RANGE_4G = 0x01,
    BNO055_CONF_ACCEL_RANGE_8G = 0x02,
    BNO055_CONF_ACCEL_RANGE_16G = 0x03,
} bno055_accel_range_t;

typedef enum
{
    BNO055_CONF_ACCEL_BANDWIDTH_7_81HZ = 0x00,
    BNO055_CONF_ACCEL_BANDWIDTH_15_63HZ = 0x04,
    BNO055_CONF_ACCEL_BANDWIDTH_31_25HZ = 0x08,
    BNO055_CONF_ACCEL_BANDWIDTH_62_5HZ = 0x0C,
    BNO055_CONF_ACCEL_BANDWIDTH_125HZ = 0x10,
    BNO055_CONF_ACCEL_BANDWIDTH_250HZ = 0x14,
    BNO055_CONF_ACCEL_BANDWIDTH_500HZ = 0x08,
    BNO055_CONF_ACCEL_BANDWIDTH_1000HZ = 0x1C
} bno055_accel_bandwidth_t;

typedef enum
{
    BNO055_CONF_ACCEL_MODE_NORMAL = 0x00,
    BNO055_CONF_ACCEL_MODE_SUSPEND = 0x20,
    BNO055_CONF_ACCEL_MODE_LOW_POWER1 = 0x40,
    BNO055_CONF_ACCEL_MODE_STANDBY = 0x60,
    BNO055_CONF_ACCEL_MODE_LOW_POWER2 = 0x80,
    BNO055_CONF_ACCEL_MODE_DEEP_SUSPEND = 0xA0
} bno055_accel_mode_t;

typedef enum
{
    BNO055_CONF_GYRO_RANGE_2000DPS = 0x00,
    BNO055_CONF_GYRO_RANGE_1000DPS = 0x01,
    BNO055_CONF_GYRO_RANGE_500DPS = 0x02,
    BNO055_CONF_GYRO_RANGE_250DPS = 0x03,
    BNO055_CONF_GYRO_RANGE_125DPS = 0x04
} bno055_gyro_range_t;

typedef enum
{
    BNO055_CONF_GYRO_BANDWIDTH_523HZ = 0x00,
    BNO055_CONF_GYRO_BANDWIDTH_230HZ = 0x08,
    BNO055_CONF_GYRO_BANDWIDTH_116HZ = 0x10,
    BNO055_CONF_GYRO_BANDWIDTH_47HZ = 0x18,
    BNO055_CONF_GYRO_BANDWIDTH_23HZ = 0x20,
    BNO055_CONF_GYRO_BANDWIDTH_12HZ = 0x28,
    BNO055_CONF_GYRO_BANDWIDTH_64HZ = 0x30,
    BNO055_CONF_GYRO_BANDWIDTH_32HZ = 0x38
} bno055_gyro_bandwidth_t;

typedef enum
{
    BNO055_CONF_GYRO_MODE_NORMAL = 0x00,
    BNO055_CONF_GYRO_MODE_FAST_PWR_UP = 0x01,
    BNO055_CONF_GYRO_MODE_DEEP_SUSPEND = 0x02,
    BNO055_CONF_GYRO_MODE_SUSPEND = 0x03,
    BNO055_CONF_GYRO_MODE_ADVANCED_PWR_SAVE = 0x04
} bno055_gyro_mode_t;

typedef enum
{
    BNO055_CONF_MAG_RATE_2HZ = 0x00,
    BNO055_CONF_MAG_RATE_6HZ = 0x01,
    BNO055_CONF_MAG_RATE_8HZ = 0x02,
    BNO055_CONF_MAG_RATE_10HZ = 0x03,
    BNO055_CONF_MAG_RATE_15HZ = 0x04,
    BNO055_CONF_MAG_RATE_20HZ = 0x05,
    BNO055_CONF_MAG_RATE_25HZ = 0x06,
    BNO055_CONF_MAG_RATE_30HZ = 0x07
} bno055_mag_rate_t;

typedef enum
{
    BNO055_CONF_MAG_MODE_LOW_PWR = 0x00,
    BNO055_CONF_MAG_MODE_REGULAR = 0x08,
    BNO055_CONF_MAG_MODE_ENHANCED_REGULAR = 0x10,
    BNO055_CONF_MAG_MODE_HIGH_ACCURACY = 0x18
} bno055_mag_mode_t;

typedef enum
{
    BNO055_CONF_MAG_PWRMODE_NORMAL = 0x00,
    BNO055_CONF_MAG_PWRMODE_SLEEP = 0x20,
    BNO055_CONF_MAG_PWRMODE_SUSPEND = 0x40,
    BNO055_CONF_MAG_PWRMODE_FORCED = 0x60
} bno055_mag_pwrmode_t;

typedef enum
{
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_4MS = 0x08,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_5MS = 0x10,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_8MS = 0x18,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_10MS = 0x20,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_15MS = 0x28,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_20MS = 0x30,
    BNO055_CONF_GYRO_AUTO_SLEEP_DURATION_40MS = 0x38
} bno055_gyro_auto_sleep_duration_t;

typedef enum
{
    BNO055_CONF_GYRO_SLEEP_DURATION_2MS = 0x00,
    BNO055_CONF_GYRO_SLEEP_DURATION_4MS = 0x01,
    BNO055_CONF_GYRO_SLEEP_DURATION_5MS = 0x02,
    BNO055_CONF_GYRO_SLEEP_DURATION_8MS = 0x03,
    BNO055_CONF_GYRO_SLEEP_DURATION_10MS = 0x04,
    BNO055_CONF_GYRO_SLEEP_DURATION_15MS = 0x05,
    BNO055_CONF_GYRO_SLEEP_DURATION_18MS = 0x06,
    BNO055_CONF_GYRO_SLEEP_DURATION_20MS = 0x07
} bno055_gyro_sleep_duration_t;

typedef enum
{
    BNO055_CONF_ACCEL_SLEEP_DURATION_0_5MS = 0x00,
    BNO055_CONF_ACCEL_SLEEP_DURATION_1MS = 0x0C,
    BNO055_CONF_ACCEL_SLEEP_DURATION_2MS = 0x0E,
    BNO055_CONF_ACCEL_SLEEP_DURATION_4MS = 0x10,
    BNO055_CONF_ACCEL_SLEEP_DURATION_6MS = 0x12,
    BNO055_CONF_ACCEL_SLEEP_DURATION_10MS = 0x14,
    BNO055_CONF_ACCEL_SLEEP_DURATION_25MS = 0x16,
    BNO055_CONF_ACCEL_SLEEP_DURATION_50MS = 0x18,
    BNO055_CONF_ACCEL_SLEEP_DURATION_100MS = 0x1A,
    BNO055_CONF_ACCEL_SLEEP_DURATION_500MS = 0x1C,
    BNO055_CONF_ACCEL_SLEEP_DURATION_1000MS = 0x1E
} bno055_accel_sleep_duration_t;

typedef enum
{
    BNO055_CONF_ACCEL_SLEEP_MODE_EVTDRIVEN = 0x00,
    BNO055_CONF_ACCEL_SLEEP_MODE_SAMPLING = 0x01
} bno055_accel_sleep_mode_t;

typedef struct
{
    int16_t accelOffsetX = 0;
    int16_t accelOffsetY = 0;
    int16_t accelOffsetZ = 0;

    int16_t magOffsetX = 0;
    int16_t magOffsetY = 0;
    int16_t magOffsetZ = 0;

    int16_t gyroOffsetX = 0;
    int16_t gyroOffsetY = 0;
    int16_t gyroOffsetZ = 0;

    int16_t accelRadius = 0;
    int16_t magRadius = 0;
} bno055_offsets_t;

typedef struct
{
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t mag = 0;
    uint8_t accel = 0;
} bno055_calibration_t;

typedef struct
{
    uint8_t accelNoSlowMotion = 0;
    uint8_t accelAnyMotion = 0;
    uint8_t accelHighG = 0;
    uint8_t gyroHR = 0;
    uint8_t gyroAnyMotion = 0;
} bno055_interrupts_status_t;

class BNO055BaseException : public std::exception
{
protected:
    std::string _msg;

public:
    BNO055BaseException(std::string message = ":-(, an error is occurred.") { _msg = message; };
    virtual const char *what() const throw() { return _msg.c_str(); }
};

class BNO055ReadFail : public BNO055BaseException
{
public:
    BNO055ReadFail(std::string message =
                       "(!*)this is specified in datasheet, but it is not in UART Application note, so it doesn't have an "
                       "official description.")
        : BNO055BaseException(message){};
};

class BNO055WriteFail : public BNO055BaseException
{
public:
    BNO055WriteFail(std::string message = "Check connection, protocol settings and operation mode of the BNO055.")
        : BNO055BaseException(message){};
};

class BNO055RegmapInvalidAddress : public BNO055BaseException
{
public:
    BNO055RegmapInvalidAddress(
        std::string message = "Check the if the register is addressable. example in Page 0, should be from 0x38 to 0x6A.")
        : BNO055BaseException(message){};
};

class BNO055RegmapWriteDisabled : public BNO055BaseException
{
public:
    BNO055RegmapWriteDisabled(std::string message = "Check the property of register.") : BNO055BaseException(message){};
};

class BNO055WrongStartByte : public BNO055BaseException
{
public:
    BNO055WrongStartByte(std::string message = "Check if the first byte sent is 0xAA.") : BNO055BaseException(message){};
};

class BNO055BusOverRunError : public BNO055BaseException
{
public:
    BNO055BusOverRunError(std::string message = "Resend the command") : BNO055BaseException(message){};
};

class BNO055MaxLengthError : public BNO055BaseException
{
public:
    BNO055MaxLengthError(std::string message = "Split the command,a single frame must have < 128 Bytes.")
        : BNO055BaseException(message){};
};

class BNO055MinLengthError : public BNO055BaseException
{
public:
    BNO055MinLengthError(std::string message = "Send a valid frame.") : BNO055BaseException(message){};
};

class BNO055ReceiveCharacterTimeout : public BNO055BaseException
{
public:
    BNO055ReceiveCharacterTimeout(std::string message = "Decrease waiting time between sending of two bytes of one frame.")
        : BNO055BaseException(message){};
};

class BNO055UnknowError : public BNO055BaseException
{
public:
    BNO055UnknowError(std::string message = ".") : BNO055BaseException(message){};
};

class BNO055UartTimeout : public BNO055BaseException
{
public:
    BNO055UartTimeout(std::string message = "timeout expired, if you see this often, try to increase timeoutMS.")
        : BNO055BaseException(message){};
};

class BNO055UartInitFailed : public BNO055BaseException
{
public:
    BNO055UartInitFailed(std::string message = "ESP32's UART Interface cannot be initialized.") : BNO055BaseException(message){};
};

class BNO055ChipNotDetected : public BNO055BaseException
{
public:
    BNO055ChipNotDetected(std::string message = "Check your wiring.") : BNO055BaseException(message){};
};

class BNO055WrongOprMode : public BNO055BaseException
{
public:
    BNO055WrongOprMode(std::string message = "Check the OperationMode.") : BNO055BaseException(message){};
};

class BNO055I2CError : public BNO055BaseException
{
public:
    BNO055I2CError(std::string message = "I2CError: Check your wiring.") : BNO055BaseException(message){};
};

class BNO055
{
public:
    BNO055(i2c_port_t i2cPort, uint8_t i2cAddr, gpio_num_t rstPin = GPIO_NUM_MAX, gpio_num_t intPin = GPIO_NUM_MAX);
    BNO055(uart_port_t uartPort, gpio_num_t txPin = GPIO_NUM_17, gpio_num_t rxPin = GPIO_NUM_16, gpio_num_t rstPin = GPIO_NUM_MAX,
           gpio_num_t intPin = GPIO_NUM_MAX);
    ~BNO055();
    void begin();
    void stop();
    void reset();

    void setPage(uint8_t page, bool forced = false);

    void setPwrModeNormal();
    void setPwrModeLowPower();
    void setPwrModeSuspend();

    void setOprModeConfig(bool forced = false);
    void setOprModeAccOnly(bool forced = false);
    void setOprModeMagOnly(bool forced = false);
    void setOprModeGyroOnly(bool forced = false);
    void setOprModeAccMag(bool forced = false);
    void setOprModeAccGyro(bool forced = false);
    void setOprModeMagGyro(bool forced = false);
    void setOprModeAMG(bool forced = false);
    void setOprModeIMU(bool forced = false);
    void setOprModeCompass(bool forced = false);
    void setOprModeM4G(bool forced = false);
    void setOprModeNdofFmcOff(bool forced = false);
    void setOprModeNdof(bool forced = false);

    void enableExternalCrystal();
    void disableExternalCrystal();

    bno055_offsets_t getSensorOffsets();
    void setSensorOffsets(bno055_offsets_t newOffsets);
    bno055_calibration_t getCalibration();

    int8_t getTemp();

    bno055_vector_t getVectorAccelerometer();
    bno055_vector_t getVectorMagnetometer();
    bno055_vector_t getVectorGyroscope();
    bno055_vector_t getVectorEuler();
    bno055_vector_t getVectorLinearAccel();
    bno055_vector_t getVectorGravity();
    bno055_quaternion_t getQuaternion();

    int16_t getSWRevision();
    uint8_t getBootloaderRevision();

    bno055_system_status_t getSystemStatus();
    bno055_self_test_result_t getSelfTestResult();
    bno055_system_error_t getSystemError();

    void setAxisRemap(bno055_axis_config_t config = BNO055_REMAP_CONFIG_P1, bno055_axis_sign_t sign = BNO055_REMAP_SIGN_P1);
    void setUnits(bno055_accel_unit_t accel = BNO055_UNIT_ACCEL_MS2,
                  bno055_angular_rate_unit_t angularRate = BNO055_UNIT_ANGULAR_RATE_RPS,
                  bno055_euler_unit_t euler = BNO055_UNIT_EULER_DEGREES, bno055_temperature_unit_t temp = BNO055_UNIT_TEMP_C,
                  bno055_data_output_format_t format = BNO055_DATA_FORMAT_ANDROID);

    void setAccelConfig(bno055_accel_range_t range = BNO055_CONF_ACCEL_RANGE_4G,
                        bno055_accel_bandwidth_t bandwidth = BNO055_CONF_ACCEL_BANDWIDTH_62_5HZ,
                        bno055_accel_mode_t mode = BNO055_CONF_ACCEL_MODE_NORMAL);
    void setGyroConfig(bno055_gyro_range_t range = BNO055_CONF_GYRO_RANGE_2000DPS,
                       bno055_gyro_bandwidth_t bandwidth = BNO055_CONF_GYRO_BANDWIDTH_32HZ,
                       bno055_gyro_mode_t mode = BNO055_CONF_GYRO_MODE_NORMAL);
    void setMagConfig(bno055_mag_rate_t rate = BNO055_CONF_MAG_RATE_20HZ,
                      bno055_mag_pwrmode_t pwrmode = BNO055_CONF_MAG_PWRMODE_FORCED,
                      bno055_mag_mode_t mode = BNO055_CONF_MAG_MODE_REGULAR);

    void setGyroSleepConfig(bno055_gyro_auto_sleep_duration_t autoSleepDuration, bno055_gyro_sleep_duration_t sleepDuration);
    void setAccelSleepConfig(bno055_accel_sleep_duration_t sleepDuration, bno055_accel_sleep_mode_t sleepMode);

    void enableAccelSlowMotionInterrupt(bool useInterruptPin = true);
    void setAccelSlowMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis = true, bool yAxis = true,
                                     bool zAxis = true);
    void disableAccelSlowMotionInterrupt();

    void enableAccelNoMotionInterrupt(bool useInterruptPin = true);
    void setAccelNoMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis = true, bool yAxis = true, bool zAxis = true);
    void disableAccelNoMotionInterrupt();

    void enableAccelAnyMotionInterrupt(bool useInterruptPin = true);
    void setAccelAnyMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis = true, bool yAxis = true, bool zAxis = true);
    void disableAccelAnyMotionInterrupt();

    void enableAccelHighGInterrupt(bool useInterruptPin = true);
    void setAccelHighGInterrupt(uint8_t threshold, uint8_t duration, bool xAxis = true, bool yAxis = true, bool zAxis = true);
    void disableAccelHighGInterrupt();

    void enableGyroAnyMotionInterrupt(bool useInterruptPin = true);
    void setGyroAnyMotionInterrupt(uint8_t threshold, uint8_t slopeSamples, uint8_t awakeDuration, bool xAxis = true,
                                   bool yAxis = true, bool zAxis = true, bool filtered = true);
    void disableGyroAnyMotionInterrupt();

    void enableGyroHRInterrupt(bool useInterruptPin = true);
    void setGyroHRInterrupt(uint8_t thresholdX, uint8_t duration, uint8_t hysteresisX, uint8_t thresholdY, uint8_t durationY,
                            uint8_t hysteresisY, uint8_t thresholdZ, uint8_t durationZ, uint8_t hysteresisZ, bool xAxis = true,
                            bool yAxis = true, bool zAxis = true, bool filtered = true);
    void disableGyroHRInterrupt();

    bno055_interrupts_status_t getInterruptsStatus();
    void clearInterruptPin();
    static void IRAM_ATTR bno055_interrupt_handler(void *arg);

    std::exception getException(uint8_t errcode);

    void i2c_readLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);
    void i2c_writeLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);

    void uart_readLen(bno055_reg_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);
    void uart_writeLen(bno055_reg_t reg, uint8_t *data, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);

    void readLen(bno055_reg_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);
    void read8(bno055_reg_t reg, uint8_t *val, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);
    void writeLen(bno055_reg_t reg, uint8_t *data, uint8_t len, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);
    void write8(bno055_reg_t reg, uint8_t val, uint32_t timeoutMS = DEFAULT_UART_TIMEOUT_MS);

    bool interruptFlag = false;

protected:
    const uart_config_t uart_config = {.baud_rate = 115200,
                                       .data_bits = UART_DATA_8_BITS,
                                       .parity = UART_PARITY_DISABLE,
                                       .stop_bits = UART_STOP_BITS_1,
                                       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                       .rx_flow_ctrl_thresh = 0,
                                       .use_ref_tick = false};

    typedef enum
    {
        BNO055_VECTOR_ACCELEROMETER = 0x08, // Default: m/s²
        BNO055_VECTOR_MAGNETOMETER = 0x0E,  // Default: uT
        BNO055_VECTOR_GYROSCOPE = 0x14,     // Default: rad/s
        BNO055_VECTOR_EULER = 0x1A,         // Default: degrees
        BNO055_VECTOR_LINEARACCEL = 0x28,   // Default: m/s²
        BNO055_VECTOR_GRAVITY = 0x2E        // Default: m/s²
    } bno055_vector_type_t;

    typedef enum
    {
        BNO055_OPERATION_MODE_CONFIG = 0x00,
        BNO055_OPERATION_MODE_ACCONLY = 0x01,
        BNO055_OPERATION_MODE_MAGONLY = 0x02,
        BNO055_OPERATION_MODE_GYRONLY = 0x03,
        BNO055_OPERATION_MODE_ACCMAG = 0x04,
        BNO055_OPERATION_MODE_ACCGYRO = 0x05,
        BNO055_OPERATION_MODE_MAGGYRO = 0x06,
        BNO055_OPERATION_MODE_AMG = 0x07,
        BNO055_OPERATION_MODE_IMU = 0x08,
        BNO055_OPERATION_MODE_COMPASS = 0x09,
        BNO055_OPERATION_MODE_M4G = 0x0A,
        BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B,
        BNO055_OPERATION_MODE_NDOF = 0x0C
    } bno055_opmode_t;

    uint8_t UART_ROUND_NUM = 64;

    gpio_num_t _rstPin;
    gpio_num_t _intPin;

    uint8_t _page;

    bool _i2cFlag;

    uart_port_t _uartPort;
    i2c_port_t _i2cPort;
    uint8_t _i2cAddr;

    gpio_num_t _txPin;
    gpio_num_t _rxPin;

    uint16_t accelScale = 100;
    uint16_t tempScale = 1;
    uint16_t angularRateScale = 16;
    uint16_t eulerScale = 16;
    uint16_t magScale = 16;

    bno055_opmode_t _mode;
    void setOprMode(bno055_opmode_t mode, bool forced = false);

    void setPwrMode(bno055_powermode_t pwrMode);

    void setExtCrystalUse(bool state);

    bno055_vector_t getVector(bno055_vector_type_t vec);
    void enableInterrupt(uint8_t flag, bool useInterruptPin = true);
    void disableInterrupt(uint8_t flag);
};

#endif