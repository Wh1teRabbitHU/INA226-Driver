#ifndef INA226_DRIVER
#define INA226_DRIVER

#include "Arduino.h"
#include "Wire.h"

#define INA226_CONFIG_ADDR      0x00
#define INA226_SHUNT_ADDR       0x01
#define INA226_BUS_ADDR         0x02
#define INA226_POWER_ADDR       0x03
#define INA226_CURRENT_ADDR     0x04
#define INA226_CALIBRATION_ADDR 0x05
#define INA226_MASK_ADDR        0x06
#define INA226_ALERT_ADDR       0x07
#define INA226_ID_ADDR          0xFF

#define INA226_SCALE_CONST      0.00512
#define INA226_MAX_RESOLUTION   32768
#define INA226_SHUNT_LSB        2.5 // uV
#define INA226_BUS_LSB          1.25 // mV

struct Measurement_t {
	uint16_t shuntVoltage;
	uint16_t busVoltage;
	uint16_t current;
	uint16_t power;
};

class INA226Driver {
	public:
		// Constructor/destructor
		INA226Driver(byte i2cSlaveAddr);
		virtual ~INA226Driver();

		// Methods
		void init(byte alertPin);
		void calibrate(byte maxCurrent, float shuntResistance);
		Measurement_t readMeasurements(void);

	private:
		byte i2cAddress;
		byte currentLSB;
		byte alertPin;

		uint16_t readFromModule(byte address);
		void writeToModule(byte address, uint16_t data);
		void writeFlag(byte address, byte pos, byte value);
		byte getValueFromBinary(byte binary, byte pos);
		byte getValueFromBinary(byte binary, byte pos, byte val);
		byte setBinary(byte binary, byte pos, byte flagVal);
};


#endif