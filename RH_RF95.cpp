// RH_RF95.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF95.cpp,v 1.18x 2018/02/22 20:55:55 gojimmypi Exp $

#include <RH_RF95.h>

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF95, allowing you to have
// 2 or more LORAs per Arduino
RH_RF95* RH_RF95::_deviceForInterrupt[RH_RF95_NUM_INTERRUPTS] = {0, 0, 0};
uint8_t RH_RF95::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
PROGMEM static const RH_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
{
	// ** IMPORTANT **  Receiver and Transmitter config must match!
	//
    // 1d,   1e,      26   
    //---   -----   -----
	// Bw125Cr45Sf128 (the chip default)
	{ 
	0x72,					// 125 kHz, Error coding rate = 4/5, Explicit Header mode
			0x74,			// sf = 7 @ 128 chips / symbol, normal mode, CRC enable, RX Time-Out MSB=0
					0x04	// no Low Data Rate Optimize, AGC enabled
	},	
    
	// Bw500Cr45Sf128 -- fast rate (shorter range); note special case of sf=6, so using sf=7 here
    { 
	0x92,					// 500 kHz, Error coding rate = 4/5, Explicit Header mode
			0x74,			// sf = 7 @ 128 chips / symbol, normal mode, CRC enable, RX Time-Out MSB=0
					 0x04	// no Low Data Rate Optimize, AGC enabled
	}, 

	// Bw31_25Cr48Sf512
    { 
	0x48,					// 31.25 kHz, Error coding rate = 4/8, Explicit Header mode
	        0x94,			// sf = 9 @ 512 chips / symbol, normal mode, CRC enable, RX Time-Out MSB=0
					0x04	// no Low Data Rate Optimize, AGC enabled
	},	

	// Bw125Cr48Sf4096
	{ 
	0x78,					// 125 kHz, Error coding rate = 4/8, Explicit Header mode
	        0xc4,			// sf = 12 @ 4096 chips / symbol, normal mode, CRC enable, RX Time-Out MSB=0
			        0x0c	// Low Data Rate Optimize, AGC enabled
	},  
    
	// Bw78Cr48Sf4096 -- note this was observed to take as long as 30 seconds to send 20 characters! (but in theory should give longest range)
	{
	0x08,					// 7.8 kHz, Error coding rate = 4/8, Explicit Header mode
			0xc4,			// sf = 12 @ 4096 chips / symbol, normal mode, CRC enable, RX Time-Out MSB=0
					0x0c	// Low Data Rate Optimize, AGC enabled
	},

};

RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin, RHGenericSPI& spi)
    :
    RHSPIDriver(slaveSelectPin, spi),
    _rxBufValid(0)
{
    _interruptPin = interruptPin;
    _myInterruptIndex = 0xff; // Not allocated yet
}

bool RH_RF95::init()
{
	uint8_t spiReadValue = 0;
	if (!RHSPIDriver::init()) {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println("RHSPIDriver::init failed!");
#endif
		return false;
	}

    // Determine the interrupt number that corresponds to the interruptPin
    int interruptNumber = digitalPinToInterrupt(_interruptPin);
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
    Serial.print("RH_RF95 interruptNumber=");
    Serial.println(interruptNumber);
#endif


    if (interruptNumber == NOT_AN_INTERRUPT) {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println("RH_RF95 NOT_AN_INTERRUPT!");
#endif
		return false;
	}
#ifdef RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER
    interruptNumber = _interruptPin;
  #if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
    Serial.print("RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER is defined!");
    Serial.print("RH_RF95 interruptNumber=");
    Serial.println(interruptNumber);
  #endif
#else
  #if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_ATTACHINTERRUPT_TAKES_PIN_NUMBER is NOT defined (typically only Arduino)");
	Serial.print("RH_RF95 interruptNumber=");
	Serial.println(interruptNumber);
  #endif
#endif

    // Tell the low level SPI interface we will use SPI within this interrupt
    spiUsingInterrupt(interruptNumber);

    // No way to check the device type :-(
    
    // Set sleep mode, so we can also set LORA mode:
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
	delay(10); // Wait for sleep mode to take over from say, CAD
	spiReadValue = spiRead(RH_RF95_REG_01_OP_MODE);
    // Check we are in sleep mode, with LORA set
	//spiReadValue = spiRead(RH_RF95_REG_01_OP_MODE);
    if (spiReadValue != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE))
    {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println("No LoRa Device Found! ");
		Serial.print(" - spiRead(RH_RF95_REG_01_OP_MODE) [1]= ");
		Serial.println(spiReadValue, HEX);
		Serial.println();
#endif
		Serial.println("RHSPIDriver::init failed during set sleep! v3");
		return false; // No device present?
    }
	else {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println("LoRa Device Found!");
		Serial.print("- spiRead(RH_RF95_REG_01_OP_MODE) [2] = ");
		Serial.println(spiRead(RH_RF95_REG_01_OP_MODE), HEX);
#endif
	}

#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
    Serial.print("RH_RF95 pinMode _interruptPin=");
    Serial.println(_interruptPin);
#endif

    // Add by Adrien van den Bossche <vandenbo@univ-tlse2.fr> for Teensy
    // ARM M4 requires the below. else pin interrupt doesn't work properly.
    // On all other platforms, its innocuous, belt and braces
    pinMode(_interruptPin, INPUT); 

    // Set up interrupt handler
    // Since there are a limited number of interrupt glue functions isr*() available,
    // we can only support a limited number of devices simultaneously
    // ON some devices, notably most Arduinos, the interrupt pin passed in is actuallt the 
    // interrupt number. You have to figure out the interruptnumber-to-interruptpin mapping
    // yourself based on knwledge of what Arduino board you are running on.
    if (_myInterruptIndex == 0xff)
    {
	// First run, no interrupt allocated yet
	if (_interruptCount <= RH_RF95_NUM_INTERRUPTS)
	    _myInterruptIndex = _interruptCount++;
	else
	    return false; // Too many devices, not enough interrupt vectors
    }
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
    Serial.print("RH_RF95 _myInterruptIndex = ");
    Serial.println(_myInterruptIndex);
#endif
    _deviceForInterrupt[_myInterruptIndex] = this;

	// static bool g_isrServiceInstalled = false;
	// If we have not yet installed the ISR service handler, install it now.
	//if (g_isrServiceInstalled == false) {
	//	ESP_LOGD(LOG_TAG, "Installing the global ISR service");
	//	esp_err_t errRc = ::gpio_install_isr_service(0);
	//	if (errRc != ESP_OK) {
	//		Serial.print(" gpio_install_isr_service error:");
	//		Serial.println(errRc);
	//	}
	//	else {
	//		Serial.println("gpio_install_isr_service complete!");
	//	}
	//	g_isrServiceInstalled = true;
	//}

	if (_myInterruptIndex == 0) {
		//void*      pArgs;
		//esp_err_t errRc = ::gpio_isr_handler_add((gpio_num_t)_interruptPin, (gpio_isr_t)isr0, pArgs);
		//if (errRc != ESP_OK) {
		//	Serial.print(" gpio_isr_handler_add error:");
		//	Serial.println(errRc);
		//}
		//else
		//{
		//	Serial.print(" gpio_isr_handler_add!");
		//	Serial.println(errRc);
		//}

		//esp_err_t rc = ::gpio_intr_enable((gpio_num_t)_interruptPin);
		//if (rc != ESP_OK) {
		//	Serial.print(" gpio_intr_enable error:");
		//	Serial.println(rc);
		//}

		//attachInterrupt(interruptNumber, handleInterruptTest, RISING);
		attachInterrupt(interruptNumber, isr0, RISING);

		//		ESP_INTR_ENABLE(interruptNumber);
//		Serial.print(" gpio_intr_enable:");
//		Serial.println(interruptNumber);
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.print("RH_RF95 isr0 attachInterrupt=");
		Serial.println(interruptNumber);
#endif
	}
	else if (_myInterruptIndex == 1)
	{
		attachInterrupt(interruptNumber, isr1, RISING);
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.print("RH_RF95 isr1 attachInterrupt=");
		Serial.println(interruptNumber);
#endif
	}
	else if (_myInterruptIndex == 2)
	{
		attachInterrupt(interruptNumber, isr2, RISING);
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.print("RH_RF95 isr2 attachInterrupt=");
		Serial.println(interruptNumber);
#endif
	}
	else
	{
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println("RH_RF95 ERROR not enough interrupt vectors!");
#endif
		return false; // Too many devices, not enough interrupt vectors
	}

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive
    // or transmit, but not both at the same time
    spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets


	setModeIdle();

	// Set up default configuration
	// No Sync Words in LORA mode.
	setModemConfig(Bw125Cr45Sf128); // Radio default
									//    setModemConfig(Bw125Cr48Sf4096); // slow and reliable?
	setPreambleLength(8); // Default is 8
						  // An innocuous ISM frequency, same as RF22's

	// set LNA boost
	// spiWrite(RH_RF95_REG_0C_LNA, spiRead(RH_RF95_REG_0C_LNA) | 0x03);

	//setFrequency(433.0);
	setFrequency(434.0);
	// Lowish power
	setTxPower(13);
	// RH_PRINT_VERSION();
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_RF95::init() complete!");
#endif
    return true;
}

// C++ level interrupt handler for this instance
// LORA is unusual in that it has several interrupt lines, and not a single, combined one.
// On MiniWirelessLoRa, only one of the several interrupt lines (DI0) from the RFM95 is usefuly 
// connnected to the processor.
// We use this to get RxDone and TxDone interrupts

//static DRAM_ATTR 
int _countInterrupt = 0;

// IRAM_ATTR // TODO is this needed for ESP32?
int RH_RF95::countInterrupt() {
	return _countInterrupt;
}

void RH_RF95::handleInterruptTest() {
	_countInterrupt++;
}

// IRAM_ATTR needed for ESP32 ?
void RH_RF95::handleInterrupt()
{
	_countInterrupt++;
	//Serial.print("RH_RF95::handleInterrupt");
	//Serial.println("!");


	// Read the interrupt register
	uint8_t irq_flags = spiRead(RH_RF95_REG_12_IRQ_FLAGS);
	if (_mode == RHModeRx && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
	{
		_rxBad++;
	}
	else if (_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
	{
		// Have received a packet
		uint8_t len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
		_bufLen = len;
		spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

		// Remember the last signal to noise ratio, LORA mode
		// Per page 111, SX1276/77/78/79 datasheet
		_lastSNR = (int8_t)spiRead(RH_RF95_REG_19_PKT_SNR_VALUE) / 4;

		// Remember the RSSI of this packet, LORA mode
		// this is according to the doc, but is it really correct?
		// weakest receiveable signals are reported RSSI at about -66
		_lastRssi = spiRead(RH_RF95_REG_1A_PKT_RSSI_VALUE);
		// Adjust the RSSI, datasheet page 87
		if (_lastSNR < 0)
			_lastRssi = _lastRssi + _lastSNR;
		else
			_lastRssi = (int)_lastRssi * 16 / 15;
		if (_usingHFport)
			_lastRssi -= 157;
		else
			_lastRssi -= 164;

		// We have received a message.
		validateRxBuf(); // this will increment _rxGood or _rxInvalid as needed
		if (_rxBufValid) 
			setModeIdle(); // Got one 
	}
	else if (_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
	{
		_txGood++;
		setModeIdle();
	}
	else if (_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
	{
		_cad = irq_flags & RH_RF95_CAD_DETECTED;
		setModeIdle();
	}
	// Sigh: on some processors, for some unknown reason, doing this only once does not actually
    // clear the radio's interrupt flag. So we do it twice. Why?
	spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags  
	spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

// These are low level functions that call the interrupt handler for the correct
// instance of RH_RF95.
// 3 interrupts allows us to have 3 different devices
void RH_RF95::isr0()
{
	if (_deviceForInterrupt[0])
		_deviceForInterrupt[0]->handleInterrupt();
}
void RH_RF95::isr1()
{
	if (_deviceForInterrupt[1])
		_deviceForInterrupt[1]->handleInterrupt();
}
void RH_RF95::isr2()
{
	if (_deviceForInterrupt[2])
		_deviceForInterrupt[2]->handleInterrupt();
}

// Check whether the latest received message is complete and uncorrupted
void RH_RF95::validateRxBuf()
{
	if (_bufLen < 4) {
		_rxBufValid = false;
		_rxInvalid++; // TODO make a rcTooShort counter
		return; // Too short to be a real message
	}
		// Extract the 4 headers
	_rxHeaderTo    = _buf[0];
	_rxHeaderFrom  = _buf[1];
	_rxHeaderId    = _buf[2];
	_rxHeaderFlags = _buf[3];
	if (_promiscuous ||
		_rxHeaderTo == _thisAddress ||
		_rxHeaderTo == RH_BROADCAST_ADDRESS)
	{
		_rxGood++;
		_rxBufValid = true;
	}
	else
	{
		_rxInvalid++;
		_rxBufValid = false;
	}
}

bool RH_RF95::available()
{
	if (_mode == RHModeTx)
		return false;
	if (_mode != RHModeRx) {
		setModeRx();
	}
	return _rxBufValid; // Will be set by the interrupt handler when a good message is received
}

void RH_RF95::clearRxBuf()
{
    ATOMIC_BLOCK_START;
    _rxBufValid = false;
    _bufLen = 0;
    ATOMIC_BLOCK_END;
}

bool RH_RF95::recv(uint8_t* buf, uint8_t* len)
{
    if (!available())
	return false;
    if (buf && len)
    {
	ATOMIC_BLOCK_START;
	// Skip the 4 headers that are at the beginning of the rxBuf
	if (*len > _bufLen-RH_RF95_HEADER_LEN)
	    *len = _bufLen-RH_RF95_HEADER_LEN;
	memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
	ATOMIC_BLOCK_END;
    }
    clearRxBuf(); // This message accepted and cleared
    return true;
}

bool RH_RF95::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN)
	return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    if (!waitCAD()) 
	return false;  // Check channel activity

    // Position at the beginning of the FIFO
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWrite(RH_RF95_REG_00_FIFO, data, len);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

bool RH_RF95::printRegisters()
{
#ifdef RH_HAVE_SERIAL 
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27};

    uint8_t i;
    for (i = 0; i < sizeof(registers); i++)
    {
	Serial.print(registers[i], HEX);
	Serial.print(": ");
	Serial.println(spiRead(registers[i]), HEX);
    }
#endif
    return true;
}

uint8_t RH_RF95::maxMessageLength()
{
    return RH_RF95_MAX_MESSAGE_LEN;
}

bool RH_RF95::setFrequency(float centre)
{
    // Frf = FRF / FSTEP
    uint32_t frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
    _usingHFport = (centre >= 779.0);

    return true;
}

void RH_RF95::setModeIdle()
{
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_RF95 Set mode idle! ");
#endif

    if (_mode != RHModeIdle)
    {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [3] ");
		int beforeReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  before write, reg = 0x");
		Serial.println(beforeReg, HEX);

		Serial.println("-spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY)");
#endif

		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
		_mode = RHModeIdle;

#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [4] ");
		int afterReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  after write, reg = 0x");
		Serial.println(afterReg, HEX);
#endif
	}
}

bool RH_RF95::sleep()
{
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_RF95 Set mode sleep! ");
#endif
    if (_mode != RHModeSleep)
    {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [5]");
		int beforeReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  before write, reg = 0x");
		Serial.println(beforeReg, HEX);

		Serial.println("-spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP)");
#endif

		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
		_mode = RHModeSleep;

#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [6]");
		int afterReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  after write, reg = 0x");
		Serial.println(afterReg, HEX);
#endif
	}
    return true;
}

void RH_RF95::setModeRx()
{
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_RF95 Set mode Rx! ");
#endif
    if (_mode != RHModeRx)
    {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.print("Current _mode = ");
		Serial.println(_mode);
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [7]");
		int beforeReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  before write, reg = 0x");
		Serial.println(beforeReg, HEX);

		Serial.println("-spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS); spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00) ");
#endif

		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
		spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone

#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [8]");
		int afterReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  after write, reg = 0x");
		Serial.println(afterReg, HEX);
#endif
		_mode = RHModeRx;
	}
}

void RH_RF95::setModeTx()
{
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
	Serial.println("RH_RF95 Set mode Tx! ");
#endif

    if (_mode != RHModeTx)
    {
#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [9]");
		int beforeReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  before write, reg = 0x");
		Serial.println(beforeReg, HEX);

		Serial.println(" - spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX); spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40) ");
#endif

		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
		spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone

#if defined(RH_HAVE_SERIAL) && (RH_DEBUG_VERBOSE >= 1)
		Serial.println(" - spiRead(RH_RF95_REG_01_OP_MODE) [10]");
		int afterReg = spiRead(RH_RF95_REG_01_OP_MODE);
		Serial.print("  after write, reg = 0x");
		Serial.println(afterReg, HEX);
#endif
		_mode = RHModeTx;
	}
}

void RH_RF95::setTxPower(int8_t power, bool useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO)
    {
	if (power > 14)
	    power = 14;
	if (power < -1)
	    power = -1;
	spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    }
    else
    {
	if (power > 23)
	    power = 23;
	if (power < 5)
	    power = 5;

	// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
	// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will us it
	// for 21, 22 and 23dBm
	if (power > 20)
	{
	    spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
	    power -= 3;
	}
	else
	{
	    spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
	}

	// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
	// pin is connected, so must use PA_BOOST
	// Pout = 2 + OutputPower.
	// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
	// but OutputPower claims it would be 17dBm.
	// My measurements show 20dBm is correct
	spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
    }
}

// See page 27 of datasheet:
//
//   SpreadingFactor    Spreading Factor   LoRa Demodulator
//  (RegModulationCfg)  (Chips / symbol)        SNR
//--------------------------------------------------------- 
//			 6				  64			- 5 dB
//			 7				 128			- 7.5 dB
//			 8				 256			- 10 dB
//			 9				 512			- 12.5 dB
//			10				1024			- 15 dB
//			11				2048			- 17.5 dB
//			12				4096			- 20 dB
//
// Note that the spreading factor, SpreadingFactor, must be known in advance on both transmit and receive sides of the link
// as different spreading factors are orthogonal to each other.Note also the resulting signal to noise ratio(SNR) required at
// the receiver input.It is the capability to receive signals with negative SNR that increases the sensitivity, so link budget and
// range, of the LoRa receiver.
//
// Spreading Factor 6
// SF = 6 Is a special use case for the highest data rate transmission possible with the LoRa modem.To this end several
// settings must be activated in the SX1276 / 77 / 78 / 79 registers when it is in use.These settings are only valid for SF6 and
// should be set back to their default values for other spreading factors :
//  - Set SpreadingFactor = 6 in RegModemConfig2
//  - The header must be set to Implicit mode.
//  - Set the bit field DetectionOptimize of register RegLoRaDetectOptimize to value "0b101".
//  - Write 0x0C in the register RegDetectionThreshold.
void RH_RF95::setSpreadingFactor(int sf)
{
	if (sf < 6) {
		sf = 6;
	}
	else if (sf > 12) {
		sf = 12;
	}

	if (sf == 6) {
		spiWrite(RH_RF95_REG_31_DETECT_OPTIMIZE, 0xc5);
		spiWrite(RH_RF95_REG_37_DETECTION_THRESHOLD, 0x0c);
	}
	else {
		spiWrite(RH_RF95_REG_31_DETECT_OPTIMIZE, 0xc3);
		spiWrite(RH_RF95_REG_37_DETECTION_THRESHOLD, 0x0a);
	}

	spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, (spiRead(RH_RF95_REG_1E_MODEM_CONFIG2) & 0x0f) | ((sf << 4) & 0xf0));
}


// get Spreading Factor resgister value
uint8_t RH_RF95::getSpreadingFactorReg() {
	return spiRead(RH_RF95_REG_1E_MODEM_CONFIG2) >> 4; // bits 7-4 
}

// get Spreading Factor
int RH_RF95::getSpreadingFactor() {
	return (1 << getSpreadingFactorReg());
}

// See page 28:
// An increase in signal bandwidth permits the use of a higher effective data rate, thus reducing transmission time at 
// the expense of reduced sensitivity improvement.
void RH_RF95::setSignalBandwidth(long sbw)
{
	int bw;

	if (sbw <= 7.8E3) {
		bw = 0;
	}
	else if (sbw <= 10.4E3) {
		bw = 1;
	}
	else if (sbw <= 15.6E3) {
		bw = 2;
	}
	else if (sbw <= 20.8E3) {
		bw = 3;
	}
	else if (sbw <= 31.25E3) {
		bw = 4;
	}
	else if (sbw <= 41.7E3) {
		bw = 5;
	}
	else if (sbw <= 62.5E3) {
		bw = 6;
	}
	else if (sbw <= 125E3) {
		bw = 7;
	}
	else if (sbw <= 250E3) {
		bw = 8;
	}
	else /*if (sbw <= 500E3)*/ {
		bw = 9; // max bandwidth is 9; 500 kHz
	}

	spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) & 0x0f) | (bw << 4));
}

// get signal bandwidth register value
uint8_t RH_RF95::getSignalBandwidthReg() {
	return spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) >> 4; // bits 7-4 
}

// get signal bandwidth, in MHz
long RH_RF95::getSignalBandwidth() {
	uint8_t bw = getSignalBandwidthReg();
	long sbw = -1;

	if (bw == 0) {
		sbw = 7.8E3;
	}
	else if (bw == 1) {
		sbw = 10.4E3;
	}
	else if (bw == 2) {
		sbw = 15.6E3;
	}
	else if (bw == 3) {
		sbw = 20.8E3;
	}
	else if (bw == 4) {
		sbw = 31.25E3;
	}
	else if (bw == 5) {
		sbw = 41.7E3;
	}
	else if (bw == 6) {
		sbw = 62.5E3;
	}
	else if (bw == 7) {
		sbw = 125E3;
	}
	else if (bw == 8) {
		sbw = 250E3;
	}
	else if (bw == 9) {
		sbw = 500E3;
	}
	else {
		sbw = -bw;
	}
	return sbw;
}


// Sets registers from a canned modem configuration structure
void RH_RF95::setModemRegisters(const ModemConfig* config)
{
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,       config->reg_1d);
    spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,       config->reg_1e);
    spiWrite(RH_RF95_REG_26_MODEM_CONFIG3,       config->reg_26);
}

// Set one of the canned FSK Modem configs
// Returns true if its a valid choice
bool RH_RF95::setModemConfig(ModemConfigChoice index)
{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
        return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return true;
}

void RH_RF95::setPreambleLength(uint16_t bytes)
{
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

bool RH_RF95::isChannelActive()
{
    // Set mode RHModeCad
    if (_mode != RHModeCad)
    {
        spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD);
        spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
        _mode = RHModeCad;
    }

    while (_mode == RHModeCad)
        YIELD;

    return _cad;
}

// see Semtech page 81 section 5.3.1. Crystal Oscillator
// Optionally, an external clock can be used to replace the crystal oscillator.This typically takes the form of 
// a tight tolerance temperature compensated crystal oscillator(TCXO).When using an external clock source the bit 
// TcxoInputOn of registerRegTcxo should be set to 1 and the external clock has to be provided on XTA(pin 5).
// XTB(pin 6) should be left open.
//
// enable TXCO (Temperature Compensated Crystal Oscillator)
void RH_RF95::enableTCXO()
{
    while ((spiRead(RH_RF95_REG_4B_TCXO) & RH_RF95_TCXO_TCXO_INPUT_ON) != RH_RF95_TCXO_TCXO_INPUT_ON)
    {
	sleep();
	spiWrite(RH_RF95_REG_4B_TCXO, (spiRead(RH_RF95_REG_4B_TCXO) | RH_RF95_TCXO_TCXO_INPUT_ON));
    } 
}

// From section 4.1.5 of SX1276/77/78/79
// Ferror = FreqError * 2**24 * BW / Fxtal / 500
int RH_RF95::frequencyError()
{
    int32_t freqerror = 0;

    // Convert 2.5 bytes (5 nibbles, 20 bits) to 32 bit signed int
    // Caution: some C compilers make errors with eg:
    // freqerror = spiRead(RH_RF95_REG_28_FEI_MSB) << 16
    // so we go more carefully.
    freqerror = spiRead(RH_RF95_REG_28_FEI_MSB);
    freqerror <<= 8;
    freqerror |= spiRead(RH_RF95_REG_29_FEI_MID);
    freqerror <<= 8;
    freqerror |= spiRead(RH_RF95_REG_2A_FEI_LSB);
    // Sign extension into top 3 nibbles
    if (freqerror & 0x80000)
	freqerror |= 0xfff00000;

    int error = 0; // In hertz
    float bw_tab[] = {7.8, 10.4, 15.6, 20.8, 31.25, 41.7, 62.5, 125, 250, 500};
    uint8_t bwindex = spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) >> 4;
    if (bwindex < (sizeof(bw_tab) / sizeof(float)))
	error = (float)freqerror * bw_tab[bwindex] * ((float)(1L << 24) / (float)RH_RF95_FXOSC / 500.0);
    // else not defined

    return error;
}

// return the last observed signal to noise ratio (SNR value)
int RH_RF95::lastSNR()
{
    return _lastSNR;
}
