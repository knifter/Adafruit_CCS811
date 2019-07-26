#ifndef __CCS811_H
#define __CCS811_H

#include <Arduino.h>
#include <math.h>
#include <TwoWireDevice.h>

// CONFIG
#define CCS811_ADDRESS_DEFAULT      (0x5A)
#define CCS811_REF_RESISTOR         100000

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with CCS811 gas sensor chips
*/
/**************************************************************************/
class CCS811 : public TwoWireDevice {
	public:
		//constructors
		CCS811(TwoWire& wire, const uint8_t addr = CCS811_ADDRESS_DEFAULT)  : TwoWireDevice(wire, addr) {};
		CCS811(const uint8_t addr = CCS811_ADDRESS_DEFAULT) : TwoWireDevice(addr) {};
        ~CCS811() {};

        // Public Data
        typedef enum
        {
            DRIVE_MODE_IDLE = 0x00,
            DRIVE_MODE_1SEC = 0x01,
            DRIVE_MODE_10SEC = 0x02,
            DRIVE_MODE_60SEC = 0x03,
            DRIVE_MODE_250MS = 0x04,
        } DriveMode_t;

        bool begin();
        void reset();

        void setEnvironmentalData(const uint8_t humidity, double temperature);

        //calculate temperature based on the NTC register
        double calculateTemperature();

        void setThresholds(const uint16_t low_med, const uint16_t med_high, const uint8_t hysteresis = 50);


        void setDriveMode(const CCS811::DriveMode_t mode);
        void enableInterrupt();
        void disableInterrupt();

        /**************************************************************************/
        /*!
            @brief  returns the stored total volatile organic compounds measurement. This does does not read the sensor. To do so, call readData()
            @returns TVOC measurement as 16 bit integer
        */
        /**************************************************************************/
		uint16_t getTVOC() { return _TVOC; }

        /**************************************************************************/
        /*!
            @brief  returns the stored estimated carbon dioxide measurement. This does does not read the sensor. To do so, call readData()
            @returns eCO2 measurement as 16 bit integer
        */
        /**************************************************************************/
		uint16_t geteCO2() { return _eCO2; }

        /**************************************************************************/
        /*!
            @brief  set the temperature compensation offset for the device. This is needed to offset errors in NTC measurements.
            @param offset the offset to be added to temperature measurements.
        */
        /**************************************************************************/
		void setTempOffset(const float offset) { _tempOffset = offset; }

		//check if data is available to be read
		bool available();
		uint8_t readData();

		bool checkError();
    protected:

        /*=========================================================================
	       REGISTER BITFIELDS
          -----------------------------------------------------------------------*/
		// The status register
        typedef struct {
            /* 0: no error
            *  1: error has occurred
            */
            uint8_t ERROR: 1;

            // reserved : 2

            /* 0: no samples are ready
            *  1: samples are ready
            */
            uint8_t DATA_READY: 1;
            uint8_t APP_VALID: 1;

			// reserved : 2

            /* 0: boot mode, new firmware can be loaded
            *  1: application mode, can take measurements
            */
            uint8_t FW_MODE: 1;

            void set(uint8_t data){
            	ERROR = data & 0x01;
            	DATA_READY = (data >> 3) & 0x01;
            	APP_VALID = (data >> 4) & 0x01;
            	FW_MODE = (data >> 7) & 0x01;
            }
        } status_t;
        status_t _status;

        //measurement and conditions register
        typedef struct {
        	// reserved : 2

        	/* 0: interrupt mode operates normally
            *  1: Interrupt mode (if enabled) only asserts the nINT signal (driven low) if the new
				ALG_RESULT_DATA crosses one of the thresholds set in the THRESHOLDS register
				by more than the hysteresis value (also in the THRESHOLDS register)
            */
        	uint8_t INT_THRESH: 1;

        	/* 0: int disabled
            *  1: The nINT signal is asserted (driven low) when a new sample is ready in
				ALG_RESULT_DATA. The nINT signal will stop being driven low when
				ALG_RESULT_DATA is read on the I²C interface.
            */
        	uint8_t INT_DATARDY: 1;

        	uint8_t DRIVE_MODE: 3;

        	uint8_t get(){
        		return (INT_THRESH << 2) | (INT_DATARDY << 3) | (DRIVE_MODE << 4);
        	}
        } meas_mode_t;
        meas_mode_t _meas_mode;

        typedef struct {
        	/* The CCS811 received an I²C write request addressed to this station but with
			invalid register address ID */
        	uint8_t WRITE_REG_INVALID: 1;

        	/* The CCS811 received an I²C read request to a mailbox ID that is invalid */
        	uint8_t READ_REG_INVALID: 1;

        	/* The CCS811 received an I²C request to write an unsupported mode to
			MEAS_MODE */
        	uint8_t MEASMODE_INVALID: 1;

        	/* The sensor resistance measurement has reached or exceeded the maximum
			range */
        	uint8_t MAX_RESISTANCE: 1;

        	/* The Heater current in the CCS811 is not in range */
        	uint8_t HEATER_FAULT: 1;

        	/*  The Heater voltage is not being applied correctly */
        	uint8_t HEATER_SUPPLY: 1;

        	void set(uint8_t data){
        		WRITE_REG_INVALID = data & 0x01;
        		READ_REG_INVALID = (data & 0x02) >> 1;
        		MEASMODE_INVALID = (data & 0x04) >> 2;
        		MAX_RESISTANCE = (data & 0x08) >> 3;
        		HEATER_FAULT = (data & 0x10) >> 4;
        		HEATER_SUPPLY = (data & 0x20) >> 5;
        	}
        } error_id_t;
        error_id_t _error_id;

        // Methods
        // bool init();

        // Other member variables
		float _tempOffset;
		uint16_t _TVOC;
		uint16_t _eCO2;

	private:
		CCS811(const CCS811&);
		CCS811& operator=(const CCS811&);
};

#endif //__CCS811_H
