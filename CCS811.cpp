#include "CCS811.h"

#include <stdio.h>
#include <math.h>

#define REG_STATUS              0x00
#define REG_MEAS_MODE           0x01
#define REG_ALG_RESULT_DATA     0x02
#define REG_RAW_DATA            0x03
#define REG_ENV_DATA            0x05
#define REG_NTC                 0x06
#define REG_THRESHOLDS          0x10
#define REG_BASELINE            0x11
#define REG_HW_ID               0x20
    #define HW_ID_CODE          0x81
#define REG_HW_VERSION          0x21
#define REG_FW_BOOT_VERSION     0x23
#define REG_FW_APP_VERSION      0x24
#define REG_SW_RESET            0xFF
#define REG_ERROR_ID            0xE0

#define BOOTLOADER_APP_ERASE    0xF1
#define BOOTLOADER_APP_DATA     0xF2
#define BOOTLOADER_APP_VERIFY   0xF3
#define BOOTLOADER_APP_START    0xF4

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is 0x5A
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool CCS811::begin()
{
	TwoWireDevice::begin();

#ifdef ESP8266
	_wire->setClockStretchLimit(500);
#endif

	reset();
	delay(100);

	//check that the HW id is correct
	if(readreg8(REG_HW_ID) != HW_ID_CODE)
		return false;

	//try to start the app
	write(BOOTLOADER_APP_START, NULL, 0);
	delay(100);

	//make sure there are no errors and we have entered application mode
	if(checkError()) 
        return false;
	if(!_status.FW_MODE) 
        return false;

	disableInterrupt();

	//default to read every second
	setDriveMode(CCS811::DRIVE_MODE_250MS);

	return true;
}

/**************************************************************************/
/*!
    @brief  sample rate of the sensor.
    @param  mode one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS.
*/
void CCS811::setDriveMode(const DriveMode_t mode)
{
	_meas_mode.DRIVE_MODE = mode;
	writereg8(REG_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*!
    @brief  enable the data ready interrupt pin on the device.
*/
/**************************************************************************/
void CCS811::enableInterrupt()
{
	_meas_mode.INT_DATARDY = 1;
	writereg8(REG_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*!
    @brief  disable the data ready interrupt pin on the device
*/
/**************************************************************************/
void CCS811::disableInterrupt()
{
	_meas_mode.INT_DATARDY = 0;
	writereg8(REG_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*!
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
bool CCS811::available()
{
	_status.set(readreg8(REG_STATUS));
	if(_status.DATA_READY)
		return true;
    return false;
}

/**************************************************************************/
/*!
    @brief  read and store the sensor data. This data can be accessed with getTVOC() and geteCO2()
    @returns 0 if no error, error code otherwise.
*/
/**************************************************************************/
uint8_t CCS811::readData()
{
	if(!available())
		return 1;

	uint8_t buf[8];
	read(REG_ALG_RESULT_DATA, buf, 8);

	_eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
	_TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);

    // TODO: _status is not updated?
	if(_status.ERROR)
		return buf[5];

	return 0;
}

/**************************************************************************/
/*!
    @brief  set the humidity and temperature compensation for the sensor.
    @param humidity the humidity data as a percentage. For 55% humidity, pass in integer 55.
    @param temperature the temperature in degrees C as a decimal number. For 25.5 degrees C, pass in 25.5
*/
/**************************************************************************/
void CCS811::setEnvironmentalData(const uint8_t humidity, double temperature)
{
	/* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
	default value is 50% = 0x64, 0x00. As an example 48.5%
	humidity would be 0x61, 0x00.*/

	/* Temperature is stored as an unsigned 16 bits integer in 1/512
	degrees; there is an offset: 0 maps to -25°C. The default value is
	25°C = 0x64, 0x00. As an example 23.5% temperature would be
	0x61, 0x00.
	The internal algorithm uses these values (or default values if
	not set by the application) to compensate for changes in
	relative humidity and ambient temperature.*/

	uint8_t hum_perc = humidity << 1;

	float fractional = modf(temperature, &temperature);
	uint16_t temp_high = (((uint16_t)temperature + 25) << 9);
	uint16_t temp_low = ((uint16_t)(fractional / 0.001953125) & 0x1FF);

	uint16_t temp_conv = (temp_high | temp_low);

	uint8_t buf[] = {hum_perc, 0x00,
		(uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)};

	write(REG_ENV_DATA, buf, 4);

}

/**************************************************************************/
/*!
    @brief  calculate the temperature using the onboard NTC resistor.
    @returns temperature as a double.
*/
/**************************************************************************/
double CCS811::calculateTemperature()
{
	uint8_t buf[4];
	read(REG_NTC, buf, 4);

	uint32_t vref = ((uint32_t)buf[0] << 8) | buf[1];
	uint32_t vntc = ((uint32_t)buf[2] << 8) | buf[3];

	//from ams ccs811 app note
	uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;

	double ntc_temp;
	ntc_temp = log((double)rntc / CCS811_REF_RESISTOR); // 1
	ntc_temp /= 3380; // 2
	ntc_temp += 1.0 / (25 + 273.15); // 3
	ntc_temp = 1.0 / ntc_temp; // 4
	ntc_temp -= 273.15; // 5
	return ntc_temp - _tempOffset;
}

/**************************************************************************/
/*!
    @brief  set interrupt thresholds
    @param low_med the level below which an interrupt will be triggered.
    @param med_high the level above which the interrupt will ge triggered.
    @param hysteresis optional histeresis level. Defaults to 50
*/
/**************************************************************************/
void CCS811::setThresholds(const uint16_t low_med, const uint16_t med_high, const uint8_t hysteresis)
{
	uint8_t buf[] = {(uint8_t)((low_med >> 8) & 0xF), (uint8_t)(low_med & 0xF),
	(uint8_t)((med_high >> 8) & 0xF), (uint8_t)(med_high & 0xF), hysteresis};

	write(REG_THRESHOLDS, buf, 5);
}

/**************************************************************************/
/*!
    @brief  trigger a software reset of the device
*/
/**************************************************************************/
void CCS811::reset()
{
	//reset sequence from the datasheet
    uint8_t seq[4] = {0x11, 0xE5, 0x72, 0x8A};
	write(REG_SW_RESET, seq, 4);
}

/**************************************************************************/
/*!
    @brief   read the status register and store any errors.
    @returns if the error bit from the status register of the device is set.
*/
/**************************************************************************/
bool CCS811::checkError()
{
	_status.set(readreg8(REG_STATUS));
	return _status.ERROR;
}
