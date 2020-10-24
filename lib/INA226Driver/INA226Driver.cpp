#include <INA226Driver.h>

INA226Driver::INA226Driver(byte i2cSlaveAddr) {
	i2cAddress = i2cSlaveAddr;
}

INA226Driver::~INA226Driver() {}

/**
 *
 * PUBLIC FUNCTIONS
 *
 */

void INA226Driver::init(byte alertPin) {
	alertPin = alertPin;
	pinMode(alertPin, INPUT);

	Wire.begin();
  	delay(40);
}

void INA226Driver::calibrate(byte maxCurrent, float shuntResistance) {
	currentLSB = maxCurrent / INA226_MAX_RESOLUTION;
	uint16_t calibrationValue = INA226_SCALE_CONST / (currentLSB * shuntResistance);

	Serial.print("Calibration value: ");
	Serial.println(calibrationValue, 16);

	writeToModule(INA226_CALIBRATION_ADDR, calibrationValue);
}

Measurement_t INA226Driver::readMeasurements() {
	uint16_t shuntVoltage = readFromModule(INA226_SHUNT_ADDR);
	uint16_t busVoltage = readFromModule(INA226_BUS_ADDR);
	uint16_t current = readFromModule(INA226_CURRENT_ADDR);
	uint16_t power = readFromModule(INA226_POWER_ADDR);

	Measurement_t measurement;

	measurement.shuntVoltage = shuntVoltage;
	measurement.busVoltage = busVoltage;
	measurement.current = current;
	measurement.power = power;

	return measurement;
}

/**
 *
 * PRIVATE FUNCTIONS
 *
 */

uint16_t INA226Driver::readFromModule(byte address) {
	Wire.beginTransmission(i2cAddress);
	Wire.write(address);
	Wire.endTransmission();
	Wire.requestFrom(i2cAddress, 2);

	if (Wire.available()) {
		return Wire.read();
	}

	return -1;
}

void INA226Driver::writeToModule(byte address, uint16_t data) {
	Wire.beginTransmission(i2cAddress);
	Wire.write(address);
	Wire.write((byte) data >> 8);
	Wire.write((byte) data | 256);
	Wire.endTransmission();
}

void INA226Driver::writeFlag(byte address, byte pos, byte value) {
	byte addressValue = readFromModule(address);
	addressValue = setBinary(addressValue, pos, value);
	writeToModule(address, addressValue);
}

byte INA226Driver::getValueFromBinary(byte binary, byte pos) {
	return getValueFromBinary(binary, pos, 1);
}

byte INA226Driver::getValueFromBinary(byte binary, byte pos, byte val) {
	return ((binary >> pos) & 1) == 1 ? val : 0;
}

byte INA226Driver::setBinary(byte binary, byte pos, byte flagVal) {
	if (flagVal == 1) {
		return binary | (1 << pos);
	}

	return binary & (~(1 << pos));
}