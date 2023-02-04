#include "flprog_SHT3X.h"

FLProgSHT3X::FLProgSHT3X(FLProgI2C *device)
{
    i2cDevice = device;
}

void FLProgSHT3X::checkDelay()
{
    if (flprog::isTimer(startDelay, sizeDelay))
    {
        step = stepAfterDelay;
    }
}

bool FLProgSHT3X::writeCommand(uint16_t command)
{
    uint8_t cmd[2];
    cmd[0] = command >> 8;
    cmd[1] = command & 0xFF;
    return i2cDevice->fullWrite(FLPROG_SHT31X_DEFAULT_ADDR, cmd, 2);
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
    codeError = i2cDevice->fullRead(FLPROG_SHT31X_DEFAULT_ADDR, data, 3);
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
    if (readPeriod > 0)
    {
        if (flprog::isTimer(startReadPeriod, readPeriod))
        {
            startReadPeriod = millis();
            read();
        }
    }
    if (step == FLPROG_SHT31X_WAITING_DELAY)
    {
        checkDelay();
    }
    if (step == FLPROG_HTU_READ_SENSOR_STEP1)
    {
        readSensor1();
    }
    if (step == FLPROG_SHT31X_WAITING_READ_STEP)
    {
        if (heaterStatus != newHeaterStatus)
        {
            setHeater();
        }
        else
        {
            if (isNeededRead)
            {
                readSensor();
                isNeededRead = false;
            }
        }
    }
}

void FLProgSHT3X::createError()
{
    startDelay = millis();
    sizeDelay = 500;
    stepAfterDelay = FLPROG_SHT31X_WAITING_READ_STEP;
    step = FLPROG_SHT31X_WAITING_DELAY;
}

void FLProgSHT3X::readSensor()
{
    codeError = writeCommand(FLPROG_SHT31X_MEAS_HIGHREP);
    if (codeError)
    {
        createError();
        return;
    }
    startDelay = millis();
    sizeDelay = 20;
    stepAfterDelay = FLPROG_HTU_READ_SENSOR_STEP1;
    step = FLPROG_SHT31X_WAITING_DELAY;
}

void FLProgSHT3X::readSensor1()
{
    uint8_t readbuffer[6];
    codeError = i2cDevice->fullRead(FLPROG_SHT31X_DEFAULT_ADDR, readbuffer, 6);
    if (codeError)
    {
        createError();
        return;
    }
    if (readbuffer[2] != crc8(readbuffer, 2) ||
        readbuffer[5] != crc8(readbuffer + 3, 2))
    {
        codeError = FLPROG_SHT31X_READ_DATA_ERROR;
        step = FLPROG_SHT31X_WAITING_READ_STEP;
        return;
    }
    int32_t stemp = (int32_t)(((uint32_t)readbuffer[0] << 8) | readbuffer[1]);
    stemp = ((4375 * stemp) >> 14) - 4500;
    temper = (float)stemp / 100.0f;
    stemp = ((uint32_t)readbuffer[3] << 8) | readbuffer[4];
    stemp = (625 * stemp) >> 12;
    hum = (float)stemp / 100.0f;
    codeError = FLPROG_SHT31X_DEVICE_NOT_ERROR;
    step = FLPROG_SHT31X_WAITING_READ_STEP;
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

void FLProgSHT3X::read()
{
    isNeededRead = true;
}

bool FLProgSHT3X::isHeaterEnabled()
{
    uint16_t regValue = readStatus();
    if (codeError)
    {
        createError();
        return;
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
    startDelay = millis();
    sizeDelay = 1;
    stepAfterDelay = FLPROG_SHT31X_WAITING_READ_STEP;
    step = FLPROG_SHT31X_WAITING_DELAY;
    heaterStatus = newHeaterStatus;
}

void FLProgSHT3X::heater(bool status)
{
    newHeaterStatus = status;
}

void FLProgSHT3X::setReadPeriod(uint32_t period)
{
    readPeriod = period;
}
