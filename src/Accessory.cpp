#include "Accessory.h"
#include <Wire.h>

Accessory::Accessory() {
	// type = NUNCHUCK;
	setControllerType(NUNCHUCK);
	this->_i2cPort = &Wire;
	I2CPortName = "Wire";
}


/**
 * Constructor which allows specifying the I2C port to use.
 * `i2cPort` should be a pointer to a `TwoWire` instance (e.g. `&Wire`, `&Wire1`, etc).
 * `portNumber` is optional, and is only used for setting the `I2CPortName` string (e.g. "Wire1").
 *   This is mostly useful for debugging purposes.	
 */
Accessory::Accessory(TwoWire* i2cPort, int8_t portNumber, int sda_pin, int scl_pin) {
	this->_i2cPort = i2cPort;
	accessoryName = "Unknown Accessory";
	I2CPortName = "Unknown I2C Port";
	if (i2cPort == &Wire) {
		I2CPortName = "Wire";
	} else if (portNumber >= 0) {
		I2CPortName = "Wire" + String(portNumber);
	}
	setControllerType(NUNCHUCK);
#   if defined(ARDUINO_ARCH_ESP32)
		_setI2CPins(sda_pin, scl_pin);
#   endif
}

/**
 * Returns the previously identified controller type.
 * (Note: Does not re-identify the controller via I2C; use `identifyController()` for that.)
 */
ControllerType Accessory::getControllerType() {
	return type;
}

/**
 * Set the controller type.
 * The controller name string will also be updated accordingly.
 */
void Accessory::setControllerType(ControllerType t) {
	type = t;
	accessoryName = _convertControllerTypeToString(t);
}


/**
 * Identifies the connected Wii Controller Type by reading its peripheral bytes.
 */
ControllerType Accessory::identifyController() {
	//TODO: Should we prevent use if there is a communication error?

	//Serial.println("Reading periph bytes");
	_burstRead(0xfa);
	//printInputs(Serial);

	if (_dataarray[4] == 0x00)
		if (_dataarray[5] == 0x00)
			return NUNCHUCK; // nunchuck

	if (_dataarray[4] == 0x01)
		if (_dataarray[5] == 0x01)
			return WIICLASSIC; // Classic Controller

	if (_dataarray[0] == 0x00)
		if (_dataarray[1] == 0x00)
			if (_dataarray[2] == 0xa4)
				if (_dataarray[3] == 0x20)
					if (_dataarray[4] == 0x01)
						if (_dataarray[5] == 0x03)
							return GuitarHeroController; // Guitar Hero Controller

	if (_dataarray[0] == 0x01)
		if (_dataarray[1] == 0x00)
			if (_dataarray[2] == 0xa4)
				if (_dataarray[3] == 0x20)
					if (_dataarray[4] == 0x01)
						if (_dataarray[5] == 0x03)
							return GuitarHeroWorldTourDrums; // Guitar Hero World Tour Drums

	if (_dataarray[0] == 0x03)
		if (_dataarray[1] == 0x00)
			if (_dataarray[2] == 0xa4)
				if (_dataarray[3] == 0x20)
					if (_dataarray[4] == 0x01)
						if (_dataarray[5] == 0x03)
							return Turntable; // Guitar Hero World Tour Drums

	if (_dataarray[0] == 0x00)
		if (_dataarray[1] == 0x00)
			if (_dataarray[2] == 0xa4)
				if (_dataarray[3] == 0x20)
					if (_dataarray[4] == 0x01)
						if (_dataarray[5] == 0x11)
							return DrumController; // Taiko no Tatsujin TaTaCon (Drum controller)

	if (_dataarray[0] == 0xFF)
		if (_dataarray[1] == 0x00)
			if (_dataarray[2] == 0xa4)
				if (_dataarray[3] == 0x20)
					if (_dataarray[4] == 0x00)
						if (_dataarray[5] == 0x13)
							return DrawsomeTablet; // Drawsome Tablet

	return UnknownChuck;
}

void Accessory::sendMultiSwitch(uint8_t iic, uint8_t sw) {
	uint8_t err = 0;
	int i = 0;
	for (; i < 10; i++) {
		this->_i2cPort->beginTransmission(iic);
		this->_i2cPort->write(1 << sw);
		this->_i2cPort->endTransmission();
		err = this->_i2cPort->endTransmission();
		if (err != 0) {
//			Serial.println(
//					"sendMultiSwitch Resetting because of " + String(err));
//			reset();
		} else
			return;
	}

}

void Accessory::addMultiplexer(uint8_t iic, uint8_t sw) {
	if (sw >= 8)
		return;

	_multiplexI2C = iic;
	_multiplexSwitch = sw;
}

void Accessory::switchMultiplexer() {
	if (_multiplexI2C == 0)
		return; // No multiplexer set
	sendMultiSwitch(_multiplexI2C, _multiplexSwitch);
}

void Accessory::switchMultiplexer(uint8_t iic, uint8_t sw) {
	if (sw >= 8)
		return;
#if defined(TWCR)
	if (TWCR == 0)
#endif
	this->_i2cPort->begin();
	// Start I2C if it's not running
	sendMultiSwitch(iic, sw);
}

/*
 * public function to read data
 */
boolean Accessory::readData() {

	if(this->_commState == ERROR){
		return false;
	}

	switchMultiplexer();

	if (_burstRead()) {
		return true;
	}
	return false;
}

/**
 * Returns the current data array.
 */
uint8_t* Accessory::getDataArray() {
	return _dataarray;
}

/**
 * This seems to initialize the Wii peripheral?
 */
void Accessory::initBytes() {
	//Serial.println("Init Periph..");
	_writeRegister(0xF0, 0x55);
	_writeRegister(0xFB, 0x00);
	delay(100);

	setControllerType(identifyController());
	delay(100);

	if (_encrypted) {
		//Serial.println("Beginning Encrypted Comms");

		delay(100);

		_writeRegister(0xF0, 0xAA); // enable enc mode?
		delay(90);

		Accessory::_burstWriteWithAddress(0x40, _key_table_1, 8);
		Accessory::_burstWriteWithAddress(0x48, _key_table_1 + 0x8, 8);
		delay(100);

		//_writeRegister(0x40, 0x00);
	}
}

void Accessory::setDataArray(uint8_t data[6]) {
	for (int i = 0; i < 6; i++)
		_dataarray[i] = data[i];
}

int Accessory::decodeInt(uint8_t mmsbbyte, uint8_t mmsbstart, uint8_t mmsbend,
		uint8_t msbbyte, uint8_t msbstart, uint8_t msbend, uint8_t csbbyte,
		uint8_t csbstart, uint8_t csbend, uint8_t lsbbyte, uint8_t lsbstart,
		uint8_t lsbend) {
// 6 bit int split across 3 bytes in 4 parts.... :(
	bool msbflag = false, csbflag = false, lsbflag = false, mmsbflag = false;
	if (msbbyte > 5)
		msbflag = true;
	if (csbbyte > 5)
		csbflag = true;
	if (lsbbyte > 5)
		lsbflag = true;
	if (mmsbbyte > 5)
		mmsbflag = true;

	uint32_t analog = 0;
	uint32_t lpart = 0;
	lpart = (lsbflag) ? 0 : _dataarray[lsbbyte];
	lpart = lpart >> lsbstart;
	lpart = lpart & (0xFF >> (7 - (lsbend - lsbstart)));

	uint32_t cpart = 0;
	cpart = (csbflag) ? 0 : _dataarray[csbbyte];
	cpart = cpart >> csbstart;
	cpart = cpart & (0xFF >> (7 - (csbend - csbstart)));

	cpart = cpart << ((lsbend - lsbstart) + 1);

	uint32_t mpart = 0;
	mpart = (msbflag) ? 0 : _dataarray[msbbyte];
	mpart = mpart >> msbstart;
	mpart = mpart & (0xFF >> (7 - (msbend - msbstart)));

	mpart = mpart << (((lsbend - lsbstart) + 1) + ((csbend - csbstart) + 1));

	uint32_t mmpart = 0;
	mmpart = (mmsbflag) ? 0 : _dataarray[mmsbbyte];
	mmpart = mmpart >> mmsbstart;
	mmpart = mmpart & (0xFF >> (7 - (mmsbend - mmsbstart)));

	mmpart = mmpart
			<< (((lsbend - lsbstart) + 1) + ((csbend - csbstart) + 1)
					+ ((msbend - msbstart) + 1));

	analog = lpart | cpart | mpart | mmpart;
	//analog = analog + offset;
	//analog = (analog*scale);

	return analog;
}

int Accessory::decodeInt(uint8_t msbbyte, uint8_t msbstart, uint8_t msbend,
		uint8_t csbbyte, uint8_t csbstart, uint8_t csbend, uint8_t lsbbyte,
		uint8_t lsbstart, uint8_t lsbend) {
// 5 bit int split across 3 bytes. what... the... fuck... nintendo...
	bool msbflag = false, csbflag = false, lsbflag = false;
	if (msbbyte > 5)
		msbflag = true;
	if (csbbyte > 5)
		csbflag = true;
	if (lsbbyte > 5)
		lsbflag = true;

	uint32_t analog = 0;
	uint16_t lpart = 0;
	lpart = (lsbflag) ? 0 : _dataarray[lsbbyte];
	lpart = lpart >> lsbstart;
	lpart = lpart & (0xFF >> (7 - (lsbend - lsbstart)));

	uint16_t cpart = 0;
	cpart = (csbflag) ? 0 : _dataarray[csbbyte];
	cpart = cpart >> csbstart;
	cpart = cpart & (0xFF >> (7 - (csbend - csbstart)));

	cpart = cpart << ((lsbend - lsbstart) + 1);

	uint16_t mpart = 0;
	mpart = (msbflag) ? 0 : _dataarray[msbbyte];
	mpart = mpart >> msbstart;
	mpart = mpart & (0xFF >> (7 - (msbend - msbstart)));

	mpart = mpart << (((lsbend - lsbstart) + 1) + ((csbend - csbstart) + 1));

	analog = lpart | cpart | mpart;
	//analog = analog + offset;
	//analog = (analog*scale);

	return analog;
}

bool Accessory::decodeBit(uint8_t byte, uint8_t bit, bool activeLow) {
	if (byte > 5)
		return false;
	uint8_t swb = _dataarray[byte];
	uint8_t sw = (swb >> bit) & 0x01;
	return activeLow ? (!sw) : (sw);
}

/**
 * Initialises the Accessory.
 * The following sequence is performed:
 *  - I2C Port is started (`_i2cPort->begin()`)
 *  - Multiplexer is switched (if configured)
 *  - Accessory is initialised via `initBytes()`
 *  - Controller type is identified via `identifyController()`
 *  - 2 burst reads are performed to prime the data
 * 
 * If clearCommunicationError is true, any existing communication error state will be cleared before attempting re-initialization.
 *   Default: false.
 */
boolean Accessory::begin(boolean clearCommunicationError) {

	if (clearCommunicationError) {
		clearCommError();
	}

	// This is mainly for internal use. It prevents us from getting in an infinite loop where `_burstRead()` calls `begin()` which then calls `_burstRead()` again.
	if (this->hasCommError() && this->_retryOnCommError == false) {
		// If we are already in a comm error state, and retry on comm error is disabled, do nothing.
		Serial.println("Unable to re-initialize " + accessoryName + "  due to previous communication error on I2C Port: " + I2CPortName);
		return commsAreOK();
	}

	//? Alternatively, instead of using the `reInit` flag, we could make it so that begin can only be called if the 
	//?   _commState is UNINITIALIZED or OK. If we are coming from an internal call from _burstRead, we should always be in an error state.
	//?   hasCommError() == True when _commState == ERROR
	
	Serial.println("Starting " + accessoryName + " on " + I2CPortName);
	this->_commState = UNINITIALIZED;


#if defined(TWCR)
	if (TWCR == 0)
#endif
#if defined(ARDUINO_ARCH_ESP32)
		this->_i2cPort->begin(SDA,SCL,10000);
#else
	this->_i2cPort->begin();

#endif
	// Start I2C if it's not running

	switchMultiplexer();

	initBytes();
	if(this->hasCommError()){
		Serial.println("Initialization failed for " + accessoryName + " on " + I2CPortName + ". Bailing Early.");
		return commsAreOK();
	}

	// If `initBytes()` was successful, Assume the rest of the calls will be successful too.
	identifyController();
	if (getControllerType() == DrawsomeTablet) {
		initBytesDrawsome();
	}
	delay(100);
	_burstRead();
	delay(100);
	_burstRead();

	return commsAreOK();
}

/**
 * Private function to burst read data from the accessory via I2C.
 * 
 */
boolean Accessory::_burstRead(uint8_t addr) {
	//int readAmnt = dataArraySize;
	uint8_t err = 0;
	bool dataBad = true;
	int b = 0;
	//bool consecCheck = true;
	uint8_t readBytes=0;

	// Loop up to 5 times to get a valid read.
	for (; b < 5; b++) {
		this->_i2cPort->beginTransmission(WII_I2C_ADDR);
		this->_i2cPort->write(addr);
		err = this->_i2cPort->endTransmission();
		if (err == 0) {			// wait for data to be converted

			delayMicroseconds(275);
			int requested = this->_i2cPort->requestFrom(WII_I2C_ADDR, dataArraySize);
			delayMicroseconds(100);
			// read data
			readBytes = this->_i2cPort->readBytes(_dataarrayTMP,requested);
			dataBad = true;
			//consecCheck=true;
			// If all bytes are 255, this is likely an error packet, reject
			for (int i = 0; i < dataArraySize && dataBad; i++){
				if(_dataarrayTMP[i]!=(uint8_t)255){
					dataBad=false;
				}
			}

			// check to see we read enough bytes and that they are valid
			if(readBytes == dataArraySize && dataBad==false){
				// decrypt bytes
				if (_encrypted) {
					for (int i = 0; i < dataArraySize; i++)
						_dataarray[i] = decryptByte(_dataarrayTMP[i], addr + i);

				}else{
					//Serial.print(" DATA= ");
					for (int i = 0; i < dataArraySize; i++){
						_dataarray[i] = _dataarrayTMP[i];
						//Serial.print(" , "+String( (uint8_t)_dataarray[i]));
					}
				}

				// ~~ Check the read in data aganst the last read date, a valid burst read is 2 reads that produce the same data ~~
				// Skip the afformentiond check against the last read data. Assume the data is good at this point.
				dataBad=false;

				// after successful read, process the data
				getValues();			//parse the data into readable data
				this->_commState = OK;
				return true; // fast return once the success case is reached

			}else{
				// We reach here if the readBytes != dataArraySize
				// OR if dataBad == True.
				dataBad=true;  // For the case where readBytes != dataArraySize, Make sure the data is marked bad
			}
		}

		if(dataBad || (err != 0) ){
			this->_commState = ERROR;  // Mark comm error true if we reach here
			if((err != 0)){
				Serial.println(	"\nI2C error code _burstRead error: " + String(err)
												+ ". repeat count: " + String(b+1));
				if(err==5){
					Serial.println("I2C error 5, re-initializing I2C bus `" + I2CPortName + "`");
					begin();
				}

			}else if(readBytes != dataArraySize){
				Serial.println("\nI2C Read length failure: " + String(readBytes) + " != " + String(dataArraySize) 
												+ " _burstRead Resetting"
												+ ". repeat count: " + String(b+1));
			}else if(dataBad){
				Serial.println("\n!!!!!! UNHANDLED _burstRead CONDITION !!!!! `dataBad` and no other applicable case."
												+ String(". repeat count: ") + String(b+1));
			}else
				Serial.println(
						"\nOther I2C error, packet all 255 _burstRead Resetting " + String(err)
								+ ". repeat count: " + String(b+1));
			reset();
		}

	}

	return !dataBad && (err == 0);
}

void Accessory::_writeRegister(uint8_t reg, uint8_t value) {
	//Serial.print("W ");
	//Serial.print(reg,HEX);
	//Serial.print(": ");
	//Serial.println(value,HEX);
	uint8_t err = 0;
	int i = 0;
	for (; i < 10; i++) {
		this->_i2cPort->beginTransmission(WII_I2C_ADDR);
		this->_i2cPort->write(reg);
		this->_i2cPort->write(value);
		err = this->_i2cPort->endTransmission();
		if (err != 0) {
//			Serial.println(
//					"_writeRegister Resetting because of " + String(err)
//							+ " repeted: " + String(i));
			this->_commState = ERROR;
			reset();
		} else{
			this->_commState = OK;
			return;
		}
	}

}

void Accessory::_burstWriteWithAddress(uint8_t addr, uint8_t* arr,
		uint8_t size) {
	//Serial.print("W ");
	//Serial.print(addr,HEX);
	//Serial.print(": ");
	//for (int i=0; i<size; i++) {//Serial.print(arr[i],HEX);//Serial.print(" ");
	//}
	//Serial.println("");
	uint8_t err = 0;
	int i = 0;
	for (; i < 3; i++) {
		this->_i2cPort->beginTransmission(WII_I2C_ADDR);
		this->_i2cPort->write(addr);
		for (int i = 0; i < size; i++)
			this->_i2cPort->write(arr[i]);
		err = this->_i2cPort->endTransmission();
		if (err != 0) {
//			Serial.println(
//					"_burstWriteWithAddress Resetting because of " + String(err)
//							+ " repeted: " + String(i));
			this->_commState = ERROR;
			reset();
		} else {
			this->_commState = OK;
			return;
		}
	}

}

void Accessory::reset() {
	if (this->hasCommError() && this->_retryOnCommError == false) {
		// If we are already in a comm error state, and retry on comm error is disabled, do nothing.
		return;
	}

	Serial.println("Resetting " + accessoryName + " on " + I2CPortName);
#if defined(ARDUINO_ARCH_ESP32)
		this->_i2cPort->begin(_sdaPin, _sclPin, 10000);
#else
	this->_i2cPort->begin();

#endif
}

void Accessory::enableEncryption(bool enc) {
	_encrypted = enc;
}

int Accessory::smap(int16_t val, int16_t aMax, int16_t aMid, int16_t aMin,
		int16_t sMax, int16_t sZero, int16_t sMin) {
	int mapv = sZero;
	if (val > aMid) {
		mapv = map(val, aMid, aMax, sZero, sMax);
	} else if (val < aMid) {
		mapv = map(val, aMin, aMid, sMin, sZero);
	}
//Serial.print(val);Serial.print(" ");Serial.println(mapv);

	return mapv;
}

uint8_t Accessory::decryptByte(uint8_t byte, uint8_t address) {
//return (byte ^ _key_table_1[address % 8]) + _key_table_1[(address % 8)+0x08];
	return (byte ^ 0x97) + 0x97;
}

void Accessory::printInputs(Stream& stream) {
	switch (getControllerType()) {
	case WIICLASSIC:
		printInputsClassic(stream);
		break;
	case GuitarHeroController:
		printInputsGuitar(stream);
		break;
	case GuitarHeroWorldTourDrums:
		printInputsDrums(stream);
		break;
	case DrumController:
		printInputsDrums(stream);
		break;
	case DrawsomeTablet:
		printInputsDrawsome(stream);
		break;
	case Turntable:
		printInputsDj(stream);
		break;
	case NUNCHUCK:
		printInputsNunchuck(stream);
		break;
	default:
		stream.println("Unknown controller!");
		break;

	}
}

uint8_t * Accessory::getValues() {
	switch (getControllerType()) {
	case WIICLASSIC:
		getValuesClassic(values);
		break;
	case GuitarHeroController:
		getValuesGuitar(values);
		break;
	case GuitarHeroWorldTourDrums:
		getValuesDrums(values);
		break;
	case DrumController:
		getValuesDrums(values);
		break;
	case DrawsomeTablet:
		getValuesDrawsome(values);
		break;
	case Turntable:
		getValuesDj(values);
		break;
	case NUNCHUCK:
	default:
		getValuesNunchuck(values);
		break;

	}
	return values;
}
;


/**
 * Convert the name of a controller type to a String.
 */
String Accessory::_convertControllerTypeToString(ControllerType type) {
	switch (type) {
	case UnknownChuck:
		return "Unknown Chuck";
	case NUNCHUCK:
		return "Nunchuck";
	case WIICLASSIC:
		return "Wii Classic Controller";
	case GuitarHeroController:
		return "Guitar Hero Controller";
	case GuitarHeroWorldTourDrums:
		return "Guitar Hero World Tour Drums";
	case DrumController:
		return "Drum Controller";
	case DrawsomeTablet:
		return "Drawsome Tablet";
	case Turntable:
		return "Turntable";
	default:
		return "Unknown Controller";
	}
}

/**
 * Returns true if there has been a communication error with the accessory.
 */
boolean Accessory::hasCommError() {
	return this->_commState == ERROR;
}

/**
 * Returns true if communication with the accessory is OK.
 *   AKA, initialized and no errors.
 */
boolean Accessory::commsAreOK() {
	return this->_commState == OK;
}

/**
 * Clears any existing communication error state.
 * This will allow the accessory to attempt to re-initialize and read data again.
 * Note: This does not re-initialize the accessory; you must call `begin()` again manually.
 */
void Accessory::clearCommError() {
	this->_commState = UNINITIALIZED;
}

/**
 * Sets whether to retry initialization and reading if there is a communication error.
 * If set to false, once a communication error occurs, the accessory will not attempt
 * to re-initialize or read data until `begin()` is called again manually.
 * Default: true
 */
void Accessory::setRetryOnCommError(boolean retry) {
	this->_retryOnCommError = retry;
}

/**
 * Returns true if the accessory will retry initialization and reading on communication errors.
 */
boolean Accessory::getRetryOnCommError() {
	return this->_retryOnCommError;
}


/**
 * Sets the pins to be used when initializing the I2C Port.
 * If the current MCU does not support specifying the pins, this function does nothing.
 */
void Accessory::_setI2CPins(int8_t sda, int8_t scl) {
	#if defined(ARDUINO_ARCH_ESP32)
		_sdaPin = sda;
		_sclPin = scl;
	#else
		// Not supported on this platform
		Serial.println("Warning: _setI2CPins() is not supported on this platform.");
	#endif
}
