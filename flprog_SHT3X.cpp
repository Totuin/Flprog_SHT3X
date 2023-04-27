#include "flprog_SHT3X.h"

FLProgSHT3X::FLProgSHT3X(AbstractFLProgI2C *device)
{
    i2cDevice = device;
    addres = 0x44;
}

bool FLProgSHT3X::writeCommand(uint16_t command)
{
    uint8_t cmd[2];
    cmd[0] = command >> 8;
    cmd[1] = command & 0xFF;
    return i2cDevice->fullWrite(addres, cmd, 2);
}

uint16_t FLProgSHT3X::readStatus()
{
    codeError = writeCommand(FLPROG_SHT31X_READSTATUS);
    if (codeError)
    {
        createError();
        return 0;
    }
    uint8_t data[3];
    codeError = i2cDevice->fullRead(addres, data, 3);
    if (codeError)
    {
        createError();
        return 0;
    }
    uint16_t stat = data[0];
    stat <<= 8;
    stat |= data[1];
    return stat;
}

void FLProgSHT3X::pool()
{
    checkReadPeriod();
    checkDelay();
    if (step == FLPROG_HTU_READ_SENSOR_STEP1)
    {
        readSensor1();
    }
    if (step == FLPROG_SENSOR_WAITING_READ_STEP)
    {
        if (heaterStatus != newHeaterStatus)
        {
            setHeater();
        }
        else
        {
            checkNeededRead();
        }
    }
}

void FLProgSHT3X::createError()
{
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 500);
}

void FLProgSHT3X::readSensor()
{
    codeError = writeCommand(FLPROG_SHT31X_MEAS_HIGHREP);
    if (codeError)
    {
        createError();
        return;
    }
    gotoStepWithDelay(FLPROG_HTU_READ_SENSOR_STEP1, 20);
}

void FLProgSHT3X::readSensor1()
{
    uint8_t readbuffer[6];
    codeError = i2cDevice->fullRead(addres, readbuffer, 6);
    if (codeError)
    {
        createError();
        return;
    }
    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2))
    {
        codeError = FLPROG_SENSOR_CRC_ERROR;
        step = FLPROG_SENSOR_WAITING_READ_STEP;
        return;
    }
    int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    stemp = ((4375 * stemp) >> 14) - 4500;
    temper = (float)stemp / 100.0f;
    stemp = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    stemp = (625 * stemp) >> 12;
    hum = (float)stemp / 100.0f;
    codeError = FLPROG_SENSOR_NOT_ERROR;
    step = FLPROG_SENSOR_WAITING_READ_STEP;
}

uint8_t FLProgSHT3X::crc8(const uint8_t *data, int len)
{

    uint8_t crc(0xFF);
    for (int j = len; j; --j)
    {
        crc ^= *data++;

        for (int i = 8; i; --i)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ FLPROG_SHT31X_POLYNOMIAL : (crc << 1);
        }
    }
    return crc;
}

bool FLProgSHT3X::isHeaterEnabled()
{
    uint16_t regValue = readStatus();
    if (codeError)
    {
        createError();
        return false;
    }
    return (bool)bitRead(regValue, FLPROG_SHT31X_REG_HEATER_BIT);
}

void FLProgSHT3X::setHeater()
{
    if (newHeaterStatus)
    {
        codeError = writeCommand(FLPROG_SHT31X_HEATEREN);
    }
    else
    {
        codeError = writeCommand(FLPROG_SHT31X_HEATERDIS);
    }
    if (codeError)
    {
        createError();
        return;
    }
    heaterStatus = newHeaterStatus;
    gotoStepWithDelay(FLPROG_SENSOR_WAITING_READ_STEP, 1);
}

void FLProgSHT3X::heater(bool status)
{
    newHeaterStatus = status;
}

