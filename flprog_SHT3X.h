#pragma once
#include "Arduino.h"
#include "flprogUtilites.h"

#define FLPROG_SHT31X_MEAS_HIGHREP_STRETCH 0x2C06 /**< Measurement High Repeatability with Clock Stretch Enabled */
#define FLPROG_SHT31X_MEAS_MEDREP_STRETCH 0x2C0D  /**< Measurement Medium Repeatability with Clock Stretch Enabled */
#define FLPROG_SHT31X_MEAS_LOWREP_STRETCH 0x2C10  /**< Measurement Low Repeatability with Clock Stretch Enabled*/
#define FLPROG_SHT31X_MEAS_HIGHREP 0x2400         /**< Measurement High Repeatability with Clock Stretch Disabled */
#define FLPROG_SHT31X_MEAS_MEDREP 0x240B          /**< Measurement Medium Repeatability with Clock Stretch Disabled */
#define FLPROG_SHT31X_MEAS_LOWREP 0x2416          /**< Measurement Low Repeatability with Clock Stretch Disabled */
#define FLPROG_SHT31X_READSTATUS 0xF32D           /**< Read Out of Status Register */
#define FLPROG_SHT31X_CLEARSTATUS 0x3041          /**< Clear Status */
#define FLPROG_SHT31X_SOFTRESET 0x30A2            /**< Soft Reset */
#define FLPROG_SHT31X_HEATEREN 0x306D             /**< Heater Enable */
#define FLPROG_SHT31X_HEATERDIS 0x3066            /**< Heater Disable */
#define FLPROG_SHT31X_REG_HEATER_BIT 0x0d         /**< Status Register Heater Bit */
#define FLPROG_SHT31X_POLYNOMIAL 0x31

#define FLPROG_HTU_READ_SENSOR_STEP1 10

class FLProgSHT3X : public FLProgI2cStepWorkSensor
{
public:
    FLProgSHT3X(FLProgI2C *device);
    void pool();
    float getHumidity() { return hum; };
    float getTemperature() { return temper; };
    bool isHeaterEnabled();
    void heater(bool status);

protected:
    uint8_t crc8(const uint8_t *data, int len);
    uint16_t readStatus();
    virtual void readSensor();
    void readSensor1();
    bool writeCommand(uint16_t command);
    void createError();
    void setHeater();
    float temper = 0;
    float hum = 0;
    bool heaterStatus = 0;
    bool newHeaterStatus = 0;
};