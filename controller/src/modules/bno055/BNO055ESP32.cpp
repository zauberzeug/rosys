// MIT License

// Copyright (c) 2019 ShellAddicted <github.com/ShellAddicted>

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

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

/*!please use the following clang-settings {BasedOnStyle: Google, ColumnLimit: 130, IndentWidth: 4}!*/
#include "BNO055ESP32.h"

/* used in ESP_LOG macros */
static const char *BNO055_LOG_TAG = "BNO055";

BNO055::BNO055(uart_port_t uartPort, gpio_num_t txPin, gpio_num_t rxPin, gpio_num_t rstPin, gpio_num_t intPin)
{
    _i2cFlag = false;

    _uartPort = uartPort;
    _txPin = txPin;
    _rxPin = rxPin;

    _rstPin = rstPin;
    _intPin = intPin;
}

BNO055::BNO055(i2c_port_t i2cPort, uint8_t i2cAddr, gpio_num_t rstPin, gpio_num_t intPin)
{
    _i2cFlag = true;

    _i2cPort = i2cPort;
    _i2cAddr = i2cAddr;

    _rstPin = rstPin;
    _intPin = intPin;
}

BNO055::~BNO055()
{
    // Free allocated resources
    // set BNO055 in supension mode to reduce power consumption
    try
    {
        setOprModeConfig();
        setPwrModeSuspend();
    }
    catch (BNO055BaseException &exc)
    {
    }
    catch (std::exception &exc)
    {
    }

    if (!_i2cFlag)
    {
        // free UART
        uart_driver_delete(_uartPort);
    }

#ifndef BNO055_DEBUG_OFF
    ESP_LOGD(BNO055_LOG_TAG, "Destroyed");
#endif
}

std::exception BNO055::getException(uint8_t errcode)
{
    if (errcode == 0x02)
    {
        return BNO055ReadFail();
    }
    else if (errcode == 0x03)
    {
        return BNO055WriteFail();
    }
    else if (errcode == 0x04)
    {
        return BNO055RegmapInvalidAddress();
    }
    else if (errcode == 0x05)
    {
        return BNO055RegmapWriteDisabled();
    }
    else if (errcode == 0x06)
    {
        return BNO055WrongStartByte();
    }
    else if (errcode == 0x07)
    {
        return BNO055BusOverRunError();
    }
    else if (errcode == 0x08)
    {
        return BNO055MaxLengthError();
    }
    else if (errcode == 0x09)
    {
        return BNO055MinLengthError();
    }
    else if (errcode == 0x0A)
    {
        return BNO055ReceiveCharacterTimeout();
    }
    else
    {
        return BNO055UnknowError();
    }
}

void BNO055::i2c_readLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS)
{
    esp_err_t errx;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, reg, ACK_EN);
    i2c_master_stop(cmd);

    for (int round = 1; round <= UART_ROUND_NUM; round++)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(i2c_RL1) Round %d", round);
#endif
        errx = i2c_master_cmd_begin(_i2cPort, cmd, timeoutMS / portTICK_PERIOD_MS);
        if (errx == ESP_OK)
        {
            break;
        }
        else if ((errx != ESP_OK) && (round < UART_ROUND_NUM))
        {
            continue;
        }
        else
        {
            i2c_cmd_link_delete(cmd);
            ESP_LOGE(BNO055_LOG_TAG, "(i2c RL) Error: %d", (int)errx);
            throw BNO055I2CError();
        }
    }
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_READ, ACK_EN);
    i2c_master_read(cmd, buffer, len, (i2c_ack_type_t)0x02);
    i2c_master_stop(cmd);
    for (int round = 1; round <= UART_ROUND_NUM; round++)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(i2c_RL2) Round %d", round);
#endif
        errx = i2c_master_cmd_begin(_i2cPort, cmd, timeoutMS / portTICK_PERIOD_MS);
        if (errx == ESP_OK)
        {
            break;
        }
        else if ((errx != ESP_OK) && (round < UART_ROUND_NUM))
        {
            continue;
        }
        else
        {
            i2c_cmd_link_delete(cmd);
            ESP_LOGE(BNO055_LOG_TAG, "(i2c RL2) Error: %d", (int)errx);
            throw BNO055I2CError();
        }
    }
    i2c_cmd_link_delete(cmd);
}

void BNO055::i2c_writeLen(uint8_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS)
{
    esp_err_t errx;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_i2cAddr << 1) | I2C_MASTER_WRITE, ACK_EN);
    i2c_master_write_byte(cmd, reg, ACK_EN);
    i2c_master_write(cmd, buffer, len, 0x01);
    i2c_master_stop(cmd);

    for (int round = 1; round <= UART_ROUND_NUM; round++)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(i2c_WL) Round %d", round);
#endif
        errx = i2c_master_cmd_begin(_i2cPort, cmd, timeoutMS / portTICK_PERIOD_MS);
        if (errx == ESP_OK)
        {
            break;
        }
        else if ((errx != ESP_OK) && (round < UART_ROUND_NUM))
        {
            continue;
        }
        else
        {
            i2c_cmd_link_delete(cmd);
            ESP_LOGE(BNO055_LOG_TAG, "(i2c WL) Error: %d", (int)errx);
            throw BNO055I2CError();
        }
    }
    i2c_cmd_link_delete(cmd);
}

void BNO055::uart_readLen(bno055_reg_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS)
{
    uint8_t res = 0;

    uint8_t cmd[4];
    cmd[0] = 0xAA; // Start Byte
    cmd[1] = 0x01; // Read
    cmd[2] = reg;
    cmd[3] = len; // len in bytes
    uint8_t *data = NULL;

    if (timeoutMS > 0)
    { // if we are expecting ack/response then allocate *data
        data = (uint8_t *)malloc(len + 2);
        if (data == NULL)
        {
            // malloc failed
            throw std::bad_alloc();
        }
    }

    for (int round = 1; round <= UART_ROUND_NUM; round++)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(RL) Round %d", round);
#endif

        // Send command over UART
        uart_flush(_uartPort);
        uart_write_bytes(_uartPort, (const char *)cmd, 4);

#ifndef BNO055_DEBUG_OFF
        ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char *)cmd, 4, ESP_LOG_DEBUG);
#endif

        if (timeoutMS == 0)
        {
            return; // Do not expect ACK/response
        }
        // else expect ACK/response

        // Read data from the UART
        int rxBytes = uart_read_bytes(_uartPort, data, (len + 2), timeoutMS / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
#ifndef BNO055_DEBUG_OFF
            ESP_LOGD(BNO055_LOG_TAG, "(RL) Read %d bytes", rxBytes);
            ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, data, rxBytes, ESP_LOG_DEBUG);
#endif

            if (data[0] == 0xBB)
            { // OK
                memcpy(buffer, data + 2,
                       len); // remove header bytes & Write back response
                free(data);
                break;
            }

            else if (data[0] == 0xEE)
            { // Error
                res = data[1];

                if ((res == 0x07 || res == 0x02 || res == 0x0A) && (round < UART_ROUND_NUM))
                {
                    continue;
                }
                ESP_LOGE(BNO055_LOG_TAG, "(RL) Error: %d", res);
                free(data);
                throw getException(res);
            }

            else
            {
                free(data);
                ESP_LOGE(BNO055_LOG_TAG, "(RL) Error: (BNO55_UNKNOW_ERROR)");
                throw BNO055UnknowError();
            }
        }
        else
        {
            free(data);
            throw BNO055UartTimeout();
        }
    }
}

void BNO055::uart_writeLen(bno055_reg_t reg, uint8_t *data2write, uint8_t len, uint32_t timeoutMS)
{
    uint8_t *cmd = (uint8_t *)malloc(len + 4);
    if (cmd == NULL)
    {
        // malloc failed
        throw std::bad_alloc();
    }
    cmd[0] = 0xAA; // Start Byte
    cmd[1] = 0x00; // Write
    cmd[2] = reg;
    cmd[3] = len; // len in bytes
    memcpy(cmd + 4, data2write, len);

    uint8_t data[2];

    // Read data from the UART
    for (int round = 1; round <= UART_ROUND_NUM; round++)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "(WL) Round %d", round); // DEBUG
#endif

        // SEND
        uart_flush(_uartPort);
        uart_write_bytes(_uartPort, (const char *)cmd, (len + 4));

#ifndef BNO055_DEBUG_OFF
        ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char *)cmd, (len + 4), ESP_LOG_DEBUG);
#endif

        if (timeoutMS == 0)
        {
            return; // Do not expect ACK/response
        }
        // else expect ACK/response

        int rxBytes = uart_read_bytes(_uartPort, data, 2, timeoutMS / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
#ifndef BNO055_DEBUG_OFF
            ESP_LOGD(BNO055_LOG_TAG, "(WL) Read %d bytes", rxBytes); // DEBUG
            ESP_LOG_BUFFER_HEXDUMP(BNO055_LOG_TAG, (const char *)data, rxBytes, ESP_LOG_DEBUG);
#endif

            if (data[0] == 0xEE)
            {
                if (data[1] == 0x01)
                { // OK
                    free(cmd);
                    break;
                }
                else if ((data[1] == 0x07 || data[1] == 0x03 || data[1] == 0x06 || data[1] == 0x0A) &&
                         (round < UART_ROUND_NUM))
                { // TRY AGAIN
                    continue;
                }
                else
                { // Error :-(
                    ESP_LOGE(BNO055_LOG_TAG, "(WL) Error: %d.", (int)data[1]);
                    free(cmd);
                    throw getException(data[1]);
                }
            }

            else
            {
                free(cmd);
                ESP_LOGE(BNO055_LOG_TAG, "(WL) Error: (BNO55_UNKNOW_ERROR)");
                throw BNO055UnknowError();
            }
        }
        else
        {
            free(cmd);
            throw BNO055UartTimeout();
        }
    }
}

void BNO055::readLen(bno055_reg_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS)
{
    if (_i2cFlag)
    {
        i2c_readLen(reg, buffer, len, timeoutMS);
    }
    else
    {
        uart_readLen(reg, buffer, len, timeoutMS);
    }
}

void BNO055::writeLen(bno055_reg_t reg, uint8_t *buffer, uint8_t len, uint32_t timeoutMS)
{
    if (!_i2cFlag)
    {
        uart_writeLen(reg, buffer, len, timeoutMS);
    }
    else
    {
        i2c_writeLen(reg, buffer, len, timeoutMS);
    }
}

void BNO055::read8(bno055_reg_t reg, uint8_t *val, uint32_t timeoutMS) { readLen(reg, val, 1, timeoutMS); }

void BNO055::write8(bno055_reg_t reg, uint8_t val, uint32_t timeoutMS) { writeLen(reg, &val, 1, timeoutMS); }

void BNO055::setPage(uint8_t page, bool forced)
{
    if (_page != page || forced == true)
    {
        write8(BNO055_REG_PAGE_ID, page);
        _page = page;
    }
}

void BNO055::setOprMode(bno055_opmode_t mode, bool forced)
{
    setPage(0);
    if (_mode != mode || forced == true)
    {
        write8(BNO055_REG_OPR_MODE, mode);
        vTaskDelay(30 / portTICK_PERIOD_MS);
        _mode = mode;
    }
}

void BNO055::setOprModeConfig(bool forced) { setOprMode(BNO055_OPERATION_MODE_CONFIG, forced); }

void BNO055::setOprModeAccOnly(bool forced) { setOprMode(BNO055_OPERATION_MODE_ACCONLY, forced); }

void BNO055::setOprModeMagOnly(bool forced) { setOprMode(BNO055_OPERATION_MODE_MAGONLY, forced); }

void BNO055::setOprModeGyroOnly(bool forced) { setOprMode(BNO055_OPERATION_MODE_GYRONLY, forced); }

void BNO055::setOprModeAccMag(bool forced) { setOprMode(BNO055_OPERATION_MODE_ACCMAG, forced); }

void BNO055::setOprModeAccGyro(bool forced) { setOprMode(BNO055_OPERATION_MODE_ACCGYRO, forced); }

void BNO055::setOprModeMagGyro(bool forced) { setOprMode(BNO055_OPERATION_MODE_MAGGYRO, forced); }

void BNO055::setOprModeAMG(bool forced) { setOprMode(BNO055_OPERATION_MODE_AMG, forced); }

void BNO055::setOprModeIMU(bool forced) { setOprMode(BNO055_OPERATION_MODE_IMU, forced); }

void BNO055::setOprModeCompass(bool forced) { setOprMode(BNO055_OPERATION_MODE_COMPASS, forced); }

void BNO055::setOprModeM4G(bool forced) { setOprMode(BNO055_OPERATION_MODE_M4G, forced); }

void BNO055::setOprModeNdofFmcOff(bool forced) { setOprMode(BNO055_OPERATION_MODE_NDOF_FMC_OFF, forced); }

void BNO055::setOprModeNdof(bool forced) { setOprMode(BNO055_OPERATION_MODE_NDOF, forced); }

void BNO055::setPwrMode(bno055_powermode_t pwrMode)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setPwrMode requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    write8(BNO055_REG_PWR_MODE, pwrMode);
}

void BNO055::setPwrModeNormal() { setPwrMode(BNO055_PWR_MODE_NORMAL); }
void BNO055::setPwrModeLowPower() { setPwrMode(BNO055_PWR_MODE_LOWPOWER); }
void BNO055::setPwrModeSuspend() { setPwrMode(BNO055_PWR_MODE_SUSPEND); }

void BNO055::setExtCrystalUse(bool state)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setExtCrystalUse requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    uint8_t tmp = 0;
    read8(BNO055_REG_SYS_TRIGGER, &tmp);
    tmp |= (state == true) ? 0x80 : 0x0;
    write8(BNO055_REG_SYS_TRIGGER, tmp);
    vTaskDelay(650 / portTICK_PERIOD_MS);
}

void BNO055::enableExternalCrystal() { setExtCrystalUse(true); }

void BNO055::disableExternalCrystal() { setExtCrystalUse(false); }

int16_t BNO055::getSWRevision()
{
    setPage(0);
    uint8_t buffer[2];
    readLen(BNO055_REG_SW_REV_ID_LSB, buffer, 2);
    return (int16_t)((buffer[1] << 8) | buffer[0]);
}

uint8_t BNO055::getBootloaderRevision()
{
    setPage(0);
    uint8_t tmp;
    read8(BNO055_REG_BL_REV_ID, &tmp);
    return tmp;
}

bno055_system_status_t BNO055::getSystemStatus()
{
    setPage(0);
    uint8_t tmp;
    read8(BNO055_REG_SYS_STATUS, &tmp);
    return (bno055_system_status_t)tmp;
}

bno055_self_test_result_t BNO055::getSelfTestResult()
{
    setPage(0);
    uint8_t tmp;
    bno055_self_test_result_t res;
    read8(BNO055_REG_ST_RESULT, &tmp);
    res.mcuState = (tmp >> 3) & 0x01;
    res.gyrState = (tmp >> 2) & 0x01;
    res.magState = (tmp >> 1) & 0x01;
    res.accState = (tmp >> 0) & 0x01;
    return res;
}

bno055_system_error_t BNO055::getSystemError()
{
    setPage(0);
    uint8_t tmp;
    read8(BNO055_REG_SYS_ERR, &tmp);
    return (bno055_system_error_t)tmp;
}

bno055_calibration_t BNO055::getCalibration()
{
    setPage(0);
    bno055_calibration_t cal;
    uint8_t calData = 0;
    read8(BNO055_REG_CALIB_STAT, &calData);
    cal.sys = (calData >> 6) & 0x03;
    cal.gyro = (calData >> 4) & 0x03;
    cal.accel = (calData >> 2) & 0x03;
    cal.mag = calData & 0x03;
    return cal;
}

int8_t BNO055::getTemp()
{
    setPage(0);
    uint8_t t;
    read8(BNO055_REG_TEMP, &t);
    t *= tempScale;
    return t;
}

void BNO055::reset()
{
    if (_rstPin == GPIO_NUM_MAX)
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "RST -> using serial bus"); // DEBUG
#endif
        write8(BNO055_REG_SYS_TRIGGER, 0x20,
               0); // RST (0 timeout because RST is not Acknowledged)
    }
    else
    {
#ifndef BNO055_DEBUG_OFF
        ESP_LOGD(BNO055_LOG_TAG, "RST -> using hardware pin"); // DEBUG
#endif
        gpio_pad_select_gpio(_rstPin);
        gpio_set_direction(_rstPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_rstPin, 0); // turn OFF
        vTaskDelay(1 / portTICK_PERIOD_MS);
        gpio_set_level(_rstPin, 1); // turn ON
    }
    vTaskDelay(700 / portTICK_PERIOD_MS); // (RE)BOOT TIME (datasheet recommends 650ms)
}

bno055_vector_t BNO055::getVector(bno055_vector_type_t vec)
{
    setPage(0);
    uint8_t buffer[6];

    /* Read (6 bytes) */
    readLen((bno055_reg_t)vec, buffer, 6);

    double scale = 1;

    if (vec == BNO055_VECTOR_MAGNETOMETER)
    {
        scale = magScale;
    }

    else if (vec == BNO055_VECTOR_ACCELEROMETER || vec == BNO055_VECTOR_LINEARACCEL || vec == BNO055_VECTOR_GRAVITY)
    {
        scale = accelScale;
    }

    else if (vec == BNO055_VECTOR_GYROSCOPE)
    {
        scale = angularRateScale;
    }

    else if (vec == BNO055_VECTOR_EULER)
    {
        scale = eulerScale;
    }

    bno055_vector_t xyz;
    xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;

    return xyz;
}

bno055_vector_t BNO055::getVectorAccelerometer() { return getVector(BNO055_VECTOR_ACCELEROMETER); }

bno055_vector_t BNO055::getVectorMagnetometer() { return getVector(BNO055_VECTOR_MAGNETOMETER); }

bno055_vector_t BNO055::getVectorGyroscope() { return getVector(BNO055_VECTOR_GYROSCOPE); }

bno055_vector_t BNO055::getVectorEuler() { return getVector(BNO055_VECTOR_EULER); }

bno055_vector_t BNO055::getVectorLinearAccel() { return getVector(BNO055_VECTOR_LINEARACCEL); }

bno055_vector_t BNO055::getVectorGravity() { return getVector(BNO055_VECTOR_GRAVITY); }

bno055_quaternion_t BNO055::getQuaternion()
{
    uint8_t buffer[8];
    double scale = 1 << 14;
    /* Read quat data (8 bytes) */
    readLen(BNO055_REG_QUA_DATA_W_LSB, buffer, 8);

    bno055_quaternion_t wxyz;
    wxyz.w = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
    wxyz.x = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
    wxyz.y = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;
    wxyz.z = (int16_t)((buffer[7] << 8) | buffer[6]) / scale;

    return wxyz;
}

bno055_offsets_t BNO055::getSensorOffsets()
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("getSensorOffsets requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    /* Accel offset range depends on the G-range:
        +/-2g  = +/- 2000 mg
        +/-4g  = +/- 4000 mg
        +/-8g  = +/- 8000 mg
        +/-1g = +/- 16000 mg
  */
    uint8_t buffer[22];
    readLen(BNO055_REG_ACC_OFFSET_X_LSB, buffer, 22);

    bno055_offsets_t sensorOffsets;
    sensorOffsets.accelOffsetX = ((buffer[1] << 8) | buffer[0]);
    sensorOffsets.accelOffsetY = ((buffer[3] << 8) | buffer[2]);
    sensorOffsets.accelOffsetZ = ((buffer[5] << 8) | buffer[4]);

    /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
    sensorOffsets.magOffsetX = ((buffer[7] << 8) | buffer[6]);
    sensorOffsets.magOffsetY = ((buffer[9] << 8) | buffer[8]);
    sensorOffsets.magOffsetZ = ((buffer[11] << 8) | buffer[10]);

    /* Gyro offset range depends on the DPS range:
        2000 dps = +/- 32000 LSB
        1000 dps = +/- 16000 LSB
        500 dps = +/- 8000 LSB
        250 dps = +/- 4000 LSB
        125 dps = +/- 2000 LSB
        ... where 1 DPS = 16 LSB
  */
    sensorOffsets.gyroOffsetX = ((buffer[13] << 8) | buffer[12]);
    sensorOffsets.gyroOffsetY = ((buffer[15] << 8) | buffer[14]);
    sensorOffsets.gyroOffsetZ = ((buffer[17] << 8) | buffer[16]);

    /* Accelerometer radius = +/- 1000 LSB */
    sensorOffsets.accelRadius = ((buffer[19] << 8) | buffer[18]);

    /* Magnetometer radius = +/- 960 LSB */
    sensorOffsets.magRadius = ((buffer[21] << 8) | buffer[20]);

    return sensorOffsets;
}

void BNO055::setSensorOffsets(bno055_offsets_t newOffsets)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setSensorOffsets requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    uint8_t offs[22];
    offs[0] = (newOffsets.accelOffsetX & 0xFF);
    offs[1] = ((newOffsets.accelOffsetX >> 8) & 0xFF);
    offs[2] = (newOffsets.accelOffsetY & 0xFF);
    offs[3] = ((newOffsets.accelOffsetY >> 8) & 0xFF);
    offs[4] = (newOffsets.accelOffsetZ & 0xFF);
    offs[5] = ((newOffsets.accelOffsetZ >> 8) & 0xFF);
    offs[6] = (newOffsets.magOffsetX & 0xFF);
    offs[7] = ((newOffsets.magOffsetX >> 8) & 0xFF);
    offs[8] = (newOffsets.magOffsetY & 0xFF);
    offs[9] = ((newOffsets.magOffsetY >> 8) & 0xFF);
    offs[10] = (newOffsets.magOffsetZ & 0xFF);
    offs[11] = ((newOffsets.magOffsetZ >> 8) & 0xFF);
    offs[12] = (newOffsets.gyroOffsetX & 0xFF);
    offs[13] = ((newOffsets.gyroOffsetX >> 8) & 0xFF);
    offs[14] = (newOffsets.gyroOffsetY & 0xFF);
    offs[15] = ((newOffsets.gyroOffsetY >> 8) & 0xFF);
    offs[16] = (newOffsets.gyroOffsetZ & 0xFF);
    offs[17] = ((newOffsets.gyroOffsetZ >> 8) & 0xFF);
    offs[18] = (newOffsets.accelRadius & 0xFF);
    offs[19] = ((newOffsets.accelRadius >> 8) & 0xFF);
    offs[20] = (newOffsets.magRadius & 0xFF);
    offs[21] = ((newOffsets.magRadius >> 8) & 0xFF);

    writeLen(BNO055_REG_ACC_OFFSET_X_LSB, offs, 22);
}

bno055_interrupts_status_t BNO055::getInterruptsStatus()
{
    setPage(0);
    uint8_t tmp = 0;
    bno055_interrupts_status_t status;
    read8(BNO055_REG_INT_STA, &tmp);
    status.gyroAnyMotion = (tmp >> 2) & 0x01;
    status.gyroHR = (tmp >> 3) & 0x01;
    status.accelHighG = (tmp >> 5) & 0x01;
    status.accelAnyMotion = (tmp >> 6) & 0x01;
    status.accelNoSlowMotion = (tmp >> 7) & 0x01;
    return status;
}

void BNO055::clearInterruptPin()
{
    setPage(0);
    interruptFlag = false;
    uint8_t tmp = 0;
    read8(BNO055_REG_SYS_TRIGGER, &tmp);
    tmp |= 0x40;
    write8(BNO055_REG_SYS_TRIGGER, tmp);
}

void IRAM_ATTR BNO055::bno055_interrupt_handler(void *arg) { static_cast<BNO055 *>(arg)->interruptFlag = true; }

void BNO055::enableInterrupt(uint8_t flag, bool useInterruptPin)
{
    uint8_t tmp[2];
    setPage(1);

    readLen(BNO055_REG_INT_MSK, tmp, 2);
    tmp[0] |= flag;
    tmp[1] = (useInterruptPin == true) ? (tmp[1] | flag) : (tmp[1] & ~flag);
    writeLen(BNO055_REG_INT_MSK, tmp, 2); // update
}

void BNO055::disableInterrupt(uint8_t flag)
{
    uint8_t tmp = 0;
    setPage(1);

    read8(BNO055_REG_INT_EN, &tmp);
    tmp &= ~flag;
    write8(BNO055_REG_INT_EN, tmp); // update
}

void BNO055::enableAccelSlowMotionInterrupt(bool useInterruptPin) { enableInterrupt(0x80, useInterruptPin); }

void BNO055::setAccelSlowMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode(
            "setAccelSlowMotionInterrupt requires "
            "BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp[2];
    setPage(1);
    tmp[0] = threshold;
    tmp[1] = ((duration << 1) | 0x00);
    writeLen(BNO055_REG_ACC_NM_THRES, tmp, 2);

    readLen(BNO055_REG_ACC_INT_SETTINGS, tmp,
            1); // read the current value to avoid overwrite of other bits
    tmp[0] = (xAxis == true) ? (tmp[0] | 0x04) : (tmp[0] & ~0x04);
    tmp[0] = (yAxis == true) ? (tmp[0] | 0x08) : (tmp[0] & ~0x08);
    tmp[0] = (zAxis == true) ? (tmp[0] | 0x10) : (tmp[0] & ~0x10);
    writeLen(BNO055_REG_ACC_INT_SETTINGS, tmp, 1); // update
}

void BNO055::disableAccelSlowMotionInterrupt() { disableInterrupt(0x80); }

void BNO055::enableAccelNoMotionInterrupt(bool useInterruptPin) { enableAccelSlowMotionInterrupt(useInterruptPin); }

void BNO055::setAccelNoMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAccelNoMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }

    uint8_t tmp[2];
    setPage(1);
    tmp[0] = threshold;
    tmp[1] = ((duration << 1) | 0x01);
    writeLen(BNO055_REG_ACC_NM_THRES, tmp, 2);

    readLen(BNO055_REG_ACC_INT_SETTINGS, tmp, 1);
    tmp[0] = (xAxis == true) ? (tmp[0] | 0x04) : (tmp[0] & ~0x04);
    tmp[0] = (yAxis == true) ? (tmp[0] | 0x08) : (tmp[0] & ~0x08);
    tmp[0] = (zAxis == true) ? (tmp[0] | 0x10) : (tmp[0] & ~0x10);
    writeLen(BNO055_REG_ACC_INT_SETTINGS, tmp, 1); // update
}

void BNO055::disableAccelNoMotionInterrupt() { disableAccelSlowMotionInterrupt(); }

void BNO055::enableAccelAnyMotionInterrupt(bool useInterruptPin) { enableInterrupt(0x40, useInterruptPin); }

void BNO055::setAccelAnyMotionInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAccelAnyMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp[2];
    setPage(1);
    tmp[0] = threshold;
    readLen(BNO055_REG_ACC_INT_SETTINGS, tmp + 1, 1);
    tmp[1] |= (duration & 0x03);
    tmp[1] = (xAxis == true) ? (tmp[1] | 0x04) : (tmp[1] & ~0x04);
    tmp[1] = (yAxis == true) ? (tmp[1] | 0x08) : (tmp[1] & ~0x08);
    tmp[1] = (zAxis == true) ? (tmp[1] | 0x10) : (tmp[1] & ~0x10);
    writeLen(BNO055_REG_ACC_AM_THRES, tmp, 2);
}

void BNO055::disableAccelAnyMotionInterrupt() { disableInterrupt(0x40); }

void BNO055::enableAccelHighGInterrupt(bool useInterruptPin) { enableInterrupt(0x20, useInterruptPin); }

void BNO055::setAccelHighGInterrupt(uint8_t threshold, uint8_t duration, bool xAxis, bool yAxis, bool zAxis)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAccelHighGInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp[3];
    setPage(1);
    readLen(BNO055_REG_ACC_INT_SETTINGS, tmp, 1);
    tmp[0] = (xAxis == true) ? (tmp[0] | 0x20) : (tmp[0] & ~0x20);
    tmp[0] = (yAxis == true) ? (tmp[0] | 0x40) : (tmp[0] & ~0x40);
    tmp[0] = (zAxis == true) ? (tmp[0] | 0x80) : (tmp[0] & ~0x80);
    tmp[1] = duration;
    tmp[2] = threshold;
    writeLen(BNO055_REG_ACC_INT_SETTINGS, tmp, 3);
}

void BNO055::disableAccelHighGInterrupt() { disableInterrupt(0x20); }

void BNO055::enableGyroAnyMotionInterrupt(bool useInterruptPin) { enableInterrupt(0x04, useInterruptPin); }

void BNO055::setGyroAnyMotionInterrupt(uint8_t threshold, uint8_t slopeSamples, uint8_t awakeDuration, bool xAxis, bool yAxis,
                                       bool zAxis, bool filtered)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setGyroAnyMotionInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp[2];
    setPage(1);
    tmp[0] = threshold;
    tmp[1] = 0x00 | (awakeDuration & 0x03);
    tmp[1] = (tmp[1] << 2) | (threshold & 0x03);
    writeLen(BNO055_REG_GYR_AM_THRES, tmp, 2);

    readLen(BNO055_REG_GYR_INT_SETTING, tmp, 1);
    tmp[0] = (xAxis == true) ? (tmp[0] | 0x01) : (tmp[0] & ~0x01);
    tmp[0] = (yAxis == true) ? (tmp[0] | 0x02) : (tmp[0] & ~0x02);
    tmp[0] = (zAxis == true) ? (tmp[0] | 0x04) : (tmp[0] & ~0x04);
    tmp[0] = (filtered == true) ? (tmp[0] & ~0x40) : (tmp[0] | 0x40);
    writeLen(BNO055_REG_GYR_INT_SETTING, tmp, 1);
}

void BNO055::disableGyroAnyMotionInterrupt() { disableInterrupt(0x04); }

void BNO055::enableGyroHRInterrupt(bool useInterruptPin) { enableInterrupt(0x08, useInterruptPin); }

void BNO055::setGyroHRInterrupt(uint8_t thresholdX, uint8_t durationX, uint8_t hysteresisX, uint8_t thresholdY, uint8_t durationY,
                                uint8_t hysteresisY, uint8_t thresholdZ, uint8_t durationZ, uint8_t hysteresisZ, bool xAxis,
                                bool yAxis, bool zAxis, bool filtered)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setGyroHRInterrupt requires BNO055_OPERATION_MODE_CONFIG");
    }
    uint8_t tmp[7];
    setPage(1);
    readLen(BNO055_REG_GYR_INT_SETTING, tmp, 1);
    tmp[0] = (xAxis == true) ? (tmp[0] | 0x01) : (tmp[0] & ~0x01);
    tmp[0] = (yAxis == true) ? (tmp[0] | 0x02) : (tmp[0] & ~0x02);
    tmp[0] = (zAxis == true) ? (tmp[0] | 0x04) : (tmp[0] & ~0x04);
    tmp[0] = (filtered == true) ? (tmp[0] & ~0x40) : (tmp[0] | 0x40);

    tmp[1] = 0x00 | (hysteresisX & 0x03);
    tmp[1] = (tmp[1] << 4) | (thresholdX & 0xF);

    tmp[2] = durationX;

    tmp[3] = 0x00 | (hysteresisY & 0x03);
    tmp[3] = (tmp[3] << 4) | (thresholdY & 0xF);

    tmp[4] = durationY;

    tmp[5] |= 0x00 | (hysteresisZ & 0x03);
    tmp[5] = (tmp[5] << 4) | (thresholdZ & 0xF);

    tmp[6] = durationZ;
    writeLen(BNO055_REG_GYR_INT_SETTING, tmp, 7);
}

void BNO055::disableGyroHRInterrupt() { disableInterrupt(0x08); }

void BNO055::setAxisRemap(bno055_axis_config_t config, bno055_axis_sign_t sign)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAxisRemap requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    uint8_t tmp[2];
    tmp[0] = ((uint8_t)config & 0x1F);
    tmp[1] = ((uint8_t)sign & 0x07);
    writeLen(BNO055_REG_AXIS_MAP_CONFIG, tmp, 2);
}

void BNO055::setUnits(bno055_accel_unit_t accel, bno055_angular_rate_unit_t angularRate, bno055_euler_unit_t euler,
                      bno055_temperature_unit_t temp, bno055_data_output_format_t format)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setUnits requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(0);
    uint8_t tmp = 0;

    tmp |= accel;
    accelScale = (accel != 0) ? 1 : 100;

    tmp |= angularRate;
    angularRateScale = (angularRate != 0) ? 900 : 16;

    tmp |= euler;
    eulerScale = (euler != 0) ? 900 : 16;

    tmp |= temp;
    tempScale = (temp != 0) ? 2 : 1;

    tmp |= format;
    write8(BNO055_REG_UNIT_SEL, tmp);
}

void BNO055::setAccelConfig(bno055_accel_range_t range, bno055_accel_bandwidth_t bandwidth, bno055_accel_mode_t mode)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAccelConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp = 0;
    tmp |= range;
    tmp |= bandwidth;
    tmp |= mode;
    write8(BNO055_REG_ACC_CONFIG, tmp);
}

void BNO055::setGyroConfig(bno055_gyro_range_t range, bno055_gyro_bandwidth_t bandwidth, bno055_gyro_mode_t mode)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setGyroConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp[2] = {0};
    tmp[0] |= range;
    tmp[0] |= bandwidth;
    tmp[1] |= mode;
    writeLen(BNO055_REG_GYR_CONFIG_0, tmp, 2);
}

void BNO055::setMagConfig(bno055_mag_rate_t rate, bno055_mag_pwrmode_t pwrmode, bno055_mag_mode_t mode)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setMagConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp = 0;
    tmp |= rate;
    tmp |= pwrmode;
    tmp |= mode;
    write8(BN0055_REG_MAG_CONFIG, tmp);
}

void BNO055::setGyroSleepConfig(bno055_gyro_auto_sleep_duration_t autoSleepDuration, bno055_gyro_sleep_duration_t sleepDuration)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setGyroSleepConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp = 0;
    tmp |= autoSleepDuration;
    tmp |= sleepDuration;
    write8(BNO055_REG_GYR_SLEEP_CONFIG, tmp);
}

void BNO055::setAccelSleepConfig(bno055_accel_sleep_duration_t sleepDuration, bno055_accel_sleep_mode_t sleepMode)
{
    if (_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        throw BNO055WrongOprMode("setAccelSleepConfig requires BNO055_OPERATION_MODE_CONFIG");
    }
    setPage(1);
    uint8_t tmp = 0;
    tmp |= sleepDuration;
    tmp |= sleepMode;
    write8(BNO055_REG_ACC_SLEEP_CONFIG, tmp);
}

void BNO055::begin()
{
    if (!_i2cFlag)
    {
        // Setup UART
        esp_err_t esperr = uart_driver_delete(_uartPort);
        uart_param_config(_uartPort, &uart_config);
        uart_set_pin(_uartPort, _txPin, _rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        esperr = uart_driver_install(_uartPort, 128 * 2, 0, 0, NULL, 0);
        if (esperr != ESP_OK)
        {
            throw BNO055UartInitFailed();
        }
    }

#ifndef BNO055_DEBUG_OFF
    ESP_LOGD(BNO055_LOG_TAG, "Setup UART -> RDY"); // DEBUG
#endif

    if (_intPin != GPIO_NUM_MAX)
    {
        gpio_pad_select_gpio(_intPin);
        gpio_set_direction(_intPin, GPIO_MODE_INPUT);
        gpio_set_intr_type(_intPin, GPIO_INTR_POSEDGE);
        gpio_set_pull_mode(_intPin, GPIO_PULLDOWN_ONLY);
        gpio_intr_enable(_intPin);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(_intPin, bno055_interrupt_handler, (void *)this);
    }
    reset();
    uint8_t id = 0;
    read8(BNO055_REG_CHIP_ID, &id);
    if (id != 0xA0)
    {
        throw BNO055ChipNotDetected(); // this is not the correct device, check
                                       // your wiring
    }
    setPage(0, true); // forced
    setOprMode(BNO055_OPERATION_MODE_CONFIG,
               true); // this should be the default OPR_MODE
    write8(BNO055_REG_SYS_TRIGGER, 0x0);
}

void BNO055::stop() { this->~BNO055(); }