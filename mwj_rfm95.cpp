// ***************************************************************************
//   RFM95 Driver
//
//     This is based on an extraction of code from the RadioHead driver
//     Extracted out items specifically for an arduino UNO 3 or 4
//     Dumbed down as much as possible!
//
//	   Combined the functions of the RHGenericDriver and the RH_RFM95
//     together to simplify things since we are only interested in the
//     RFM95 device. 
//
// Extracted from code originally developed by:
//
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2014 Mike McCauley
// $Id: RH_RF95.h,v 1.26 2020/06/15 23:39:39 mikem Exp $
//
// Pin connections
//
//                 Arduino      RFM95
//                 GND----------GND   (ground in)
//                 3V3----------3.3V  (3.3V in)
// interrupt 0 pin D2-----------DIO0  (interrupt request out)
//          SS pin D10----------NSS   (CS chip select in)
//         SCK pin D13----------SCK   (SPI clock in)
//        MOSI pin D11----------MOSI  (SPI Data in)
//        MISO pin D12----------MISO  (SPI Data out)
//
//
// All messages sent and received by this Driver conform to this packet format:
//
// - LoRa mode:
// - 8 symbol PREAMBLE
// - Explicit header with header CRC (default CCITT, handled internally by the radio)
// - 4 octets HEADER: (TO, FROM, ID, FLAGS)
// - 0 to 251 octets DATA 
// - CRC (default CCITT, handled internally by the radio)
//
//  Memory
//
// The RH_RF95 driver requires non-trivial amounts of memory. The sample
// programs all compile to about 8kbytes each, which will fit in the
// flash proram memory of most Arduinos. However, the RAM requirements are
// more critical. Therefore, you should be vary sparing with RAM use in
// programs that use the RH_RF95 driver.
//
// It is often hard to accurately identify when you are hitting RAM limits on Arduino. 
// The symptoms can include:
// - Mysterious crashes and restarts
// - Changes in behaviour when seemingly unrelated changes are made (such as adding print() statements)
// - Hanging
// - Output from Serial.print() not appearing
//
// Range
//
// We have made some simple range tests under the following conditions:
// - rf95_client base station connected to a VHF discone antenna at 8m height above ground
// - rf95_server mobile connected to 17.3cm 1/4 wavelength antenna at 1m height, no ground plane.
// - Both configured for 13dBm, 434MHz, Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, CRC on. Slow+long range
// - Minimum reported RSSI seen for successful comms was about -91
// - Range over flat ground through heavy trees and vegetation approx 2km.
// - At 20dBm (100mW) otherwise identical conditions approx 3km.
// - At 20dBm, along salt water flat sandy beach, 3.2km.
//
// It should be noted that at this data rate, a 12 octet message takes 2 seconds to transmit.
//
// At 20dBm (100mW) with Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. 
// (Default medium range) in the conditions described above.
// - Range over flat ground through heavy trees and vegetation approx 2km.
//
// Caution: the performance of this radio, especially with narrow bandwidths is strongly dependent on the
// accuracy and stability of the chip clock. HopeRF and Semtech do not appear to 
// recommend bandwidths of less than 62.5 kHz 
// unless you have the optional Temperature Compensated Crystal Oscillator (TCXO) installed and 
// enabled on your radio module. See the refernece manual for more data.
// Also https://lowpowerlab.com/forum/rf-range-antennas-rfm69-library/lora-library-experiences-range/15/
// and http://www.semtech.com/images/datasheet/an120014-xo-guidance-lora-modulation.pdf
// ***************************************************************************

#include<mwj_rfm95.h>

RH_RF95* RH_RF95::_deviceForInterrupt=0;

// These are indexed by the values of ModemConfigChoice Stored in flash (program) memory to save SRAM
PROGMEM static const RH_RF95::ModemConfig MODEM_CONFIG_TABLE[] =
	{
		//  1d,     1e,      26
		{ 0x72,   0x74,    0x04}, 	// Bw125Cr45Sf128 (the chip default), AGC enabled
		{ 0x92,   0x74,    0x04}, 	// Bw500Cr45Sf128, AGC enabled
		{ 0x48,   0x94,    0x04}, 	// Bw31_25Cr48Sf512, AGC enabled
		{ 0x78,   0xc4,    0x0c}, 	// Bw125Cr48Sf4096, AGC enabled
		{ 0x72,   0xb4,    0x04}, 	// Bw125Cr45Sf2048, AGC enabled
    
	};

// ***************************************
// constructor
// ***************************************
RH_RF95::RH_RF95(uint8_t slaveSelectPin, uint8_t interruptPin) 
	{
    _slaveSelectPin=slaveSelectPin;
	_rxBufValid=false;
    _interruptPin=interruptPin;
    _enableCRC=true;
    _useRFO=false;

    _mode=RHModeInitialising;
    _thisAddress=RH_BROADCAST_ADDRESS;
    _txHeaderTo=RH_BROADCAST_ADDRESS;
    _txHeaderFrom=RH_BROADCAST_ADDRESS;
    _txHeaderId=0;
    _txHeaderFlags=0;
    _rxBad=0;
    _rxGood=0;
    _txGood=0;
    _cad_timeout=0;
	}


// ***************************************
//
//  Initialize the RFM95 and associated
//  sub-systems
//
// ***************************************
bool RH_RF95::init()
	{
    int interruptNumber;

	_deviceForInterrupt=this;



	//Set up interrupts from the RFM95 device. 
	//Set the pin a digital input, determine the associated interrupt number, attach an interrupt service routine
    if (_interruptPin == RH_INVALID_PIN) return false;
		
	pinMode(_interruptPin, INPUT);
	interruptNumber=digitalPinToInterrupt(_interruptPin);
	if(interruptNumber==-1) return false;
	attachInterrupt(interruptNumber,isr0,RISING);
   
    //Initialize the SPI system. We will use SPI within the RFM95 interrupts
    spiInit();
	spiUsingInterrupt(interruptNumber);
    interrupts();

    // Set device to sleep mode and configure into LORA mode:
	// Wait for sleep mode to take over from say, CAD
    spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE);
    delay(10); 

    // Check we are in sleep mode, with LORA set - if not then no device available
    if (spiRead(RH_RF95_REG_01_OP_MODE) != (RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE)) return false;

    // Set up FIFO
    // We configure so that we can use the entire 256 byte FIFO for either receive or transmit, but not both at the same time
    spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
    spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

    // Packet format is preamble + explicit-header + payload + crc
    // Explicit Header Mode
    // payload is TO + FROM + ID + FLAGS + message data
    // RX mode is implmented with RXCONTINUOUS
    // max message data length is 255 - 4 = 251 octets

    setModeIdle();

    // Set up default configuration
    setModemConfig(Bw125Cr45Sf128); 	// Radio configuration
    setPreambleLength(8); 				// Default is 8
    setFrequency(434.0);
    setTxPower(13);

    return true;
	}


// ********************************************************
//   isr0()
//
//     Function to process interrupts.
//     This must be static so that it can be determined
//     early.
//
// ********************************************************
void RH_RF95::isr0()
	{
	_deviceForInterrupt->handleInterrupt();
	}


// ********************************************************
//   handleInterrupt()
//
//     Process interrupts from the RFM95
//
// ********************************************************
void RH_RF95::handleInterrupt()
	{
	uint8_t irq_flags;
    uint8_t hop_channel;
	uint8_t len;

	//get reason for IRQ
	irq_flags=spiRead(RH_RF95_REG_12_IRQ_FLAGS);

	// Read the RegHopChannel register to check if CRC presence is signalled in the header. If not it might be a stray (noise) packet.*
    hop_channel=spiRead(RH_RF95_REG_1C_HOP_CHANNEL);

    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

    // error if: timeout bad CRC, CRC is required but it is not present
    if(_mode == RHModeRx
		&& ((irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
	    || (_enableCRC && !(hop_channel & RH_RF95_RX_PAYLOAD_CRC_IS_ON)) ))
		{
		_rxBad++;
        clearRxBuf();
		}

    // It is possible to get RX_DONE and CRC_ERROR and VALID_HEADER all at once so this must be an else
    else if(_mode == RHModeRx && irq_flags & RH_RF95_RX_DONE)
		{
		// Packet received, no CRC error. Have received a packet
		len = spiRead(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, spiRead(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		spiBurstRead(RH_RF95_REG_00_FIFO, _buf, len);
		_bufLen = len;

		// Remember the last signal to noise ratio, LORA mode Per page 111, SX1276/77/78/79 datasheet
		_lastSNR = (int8_t)spiRead(RH_RF95_REG_19_PKT_SNR_VALUE) / 4;

		// Remember the RSSI of this packet, LORA mode this is according to the doc, but is it really correct? weakest receiveable signals are reported RSSI at about -66
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
		validateRxBuf(); 
		if (_rxBufValid) setModeIdle(); // Got one 
		}
    
	else if(_mode == RHModeTx && irq_flags & RH_RF95_TX_DONE)
		{
		_txGood++;
		setModeIdle();
		}
	else if(_mode == RHModeCad && irq_flags & RH_RF95_CAD_DONE)
		{
        _cad = irq_flags & RH_RF95_CAD_DETECTED;
        setModeIdle();
		}
    else
		{
		}
	
    // Sigh: on some processors, for some unknown reason, doing this only once does not actually clear the radio's interrupt flag. So we do it twice. Why?
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
    spiWrite(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
	}


// ***************************************************************************
//   validateRxBuf()
// 
//     Check whether the latest received message is complete and uncorrupted
//
// ***************************************************************************
void RH_RF95::validateRxBuf()
	{
    if (_bufLen < 4) return; // Too short to be a real message

    // Extract the 4 headers
    _rxHeaderTo    = _buf[0];
    _rxHeaderFrom  = _buf[1];
    _rxHeaderId    = _buf[2];
    _rxHeaderFlags = _buf[3];

    if (_promiscuous || _rxHeaderTo == _thisAddress || _rxHeaderTo == RH_BROADCAST_ADDRESS)
		{
		_rxGood++;
		_rxBufValid = true;
		}

	}


// ***************************************************************************
//   available()
//
//     determine if frame available
//
// ****************************************************************************
bool RH_RF95::available()
	{
    if(_mode == RHModeTx) return false;
    
    setModeRx();
    return _rxBufValid; // Will be set by the interrupt handler when a good message is received
	}


// ***************************************************************************
//   clearRxBuf()
//
//     clear the rx buffer
//
// ***************************************************************************
void RH_RF95::clearRxBuf()
	{
    _rxBufValid = false;
    _bufLen = 0;
	}

// ***************************************************************************
//   recv()
//
//     retrieve frame from rx buffer
//
// ***************************************************************************
bool RH_RF95::recv(uint8_t* buf, uint8_t* len)
	{
    if (!available()) return false;

    if (buf && len)
		{
		// Skip the 4 headers that are at the beginning of the rxBuf
		if (*len > _bufLen-RH_RF95_HEADER_LEN)
		*len = _bufLen-RH_RF95_HEADER_LEN;
		memcpy(buf, _buf+RH_RF95_HEADER_LEN, *len);
		}
    
	clearRxBuf();
    return true;
	}


// ***************************************************************************
//   send()
//
//     Setup a message for transmit
//
// ***************************************************************************
bool RH_RF95::send(const uint8_t* data, uint8_t len)
{
    if (len > RH_RF95_MAX_MESSAGE_LEN) return false;

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    if (!waitCAD()) return false;  // Check channel activity

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


// ***************************************************************************
//   printRegisters()
//
//    Prints various registers
//
// ***************************************************************************
bool RH_RF95::printRegisters()
	{
    uint8_t registers[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, 0x11, 0x12, 0x13, 0x014, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x4b};
    uint8_t i;

    for (i = 0; i < sizeof(registers); i++)
		{
		Serial.print(registers[i], HEX);
		Serial.print(": ");
		Serial.println(spiRead(registers[i]), HEX);
		}

    return true;
	}


// ***************************************************************************
//   maxMessageLength()
//
//     returns the maximuim message length
//
// ***************************************************************************
uint8_t RH_RF95::maxMessageLength()
	{
    return RH_RF95_MAX_MESSAGE_LEN;
	}


// ***************************************************************************
//   setFrequency()
//
//     set the frequency
//
// ***************************************************************************
bool RH_RF95::setFrequency(float centre)
	{
	uint32_t frf;

    // Frf = FRF / FSTEP
    frf = (centre * 1000000.0) / RH_RF95_FSTEP;
    spiWrite(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
    spiWrite(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
    spiWrite(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
    _usingHFport = (centre >= 779.0);

    return true;
	}


// ***************************************************************************
//   setModeIdle()
//
//     put device into idle mode
//
// ***************************************************************************
void RH_RF95::setModeIdle()
	{
    if (_mode != RHModeIdle)
		{
		modeWillChange(RHModeIdle);
		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
		_mode = RHModeIdle;
		}
	}


// ***************************************************************************
//   sleep()
//
//     put device into sleep mode
//
// ***************************************************************************
bool RH_RF95::sleep()
	{
    if (_mode != RHModeSleep)
		{
		modeWillChange(RHModeSleep);
		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
		_mode = RHModeSleep;
		}
    return true;
	}


// ***************************************************************************
//   setModeRx()
//
//     Put device into rx mode
//     Enable interrupt on Rx done
//
// ***************************************************************************
void RH_RF95::setModeRx()
	{
	if (_mode != RHModeRx)
		{
		modeWillChange(RHModeRx);
		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
		spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
		_mode = RHModeRx;
		}
	}


// ***************************************************************************
//   setModeTTx()
//
//     put device into tx mode
//     Enable interrupt on Tx done
//
// ***************************************************************************
void RH_RF95::setModeTx()
	{
    if (_mode != RHModeTx)
		{
		modeWillChange(RHModeTx);
		spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
		spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
		_mode = RHModeTx;
		}
	}


// ***************************************************************************
//   setTxPower()
//
//     set transmit power
//
// ***************************************************************************
void RH_RF95::setTxPower(int8_t power, bool useRFO)
	{
    _useRFO = useRFO;
    
    // Sigh, different behaviours depending on whether the module use PA_BOOST or the RFO pin for the transmitter output
    if (useRFO)
		{
		if (power > 15) power = 15;
		if (power < 0) power = 0;

		// Set the MaxPower register to 0x7 => MaxPower = 10.8 + 0.6 * 7 = 15dBm So Pout = Pmax - (15 - power) = 15 - 15 + power
		spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | power);
		spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
		}
    else
		{
		if (power > 20) power = 20;
		if (power < 2) power = 2;

		// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
		// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it for 8, 19 and 20dBm
		if (power > 17)
			{
			spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
			power -= 3;
			}
		else
			{
			spiWrite(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
			}

		// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
		// pin is connected, so must use PA_BOOST, Pout = 2 + OutputPower (+3dBm if DAC enabled)
		spiWrite(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-2));
		}
	}


// ***************************************************************************
//   setModemRegisters()
//
//     Sets registers from a canned modem configuration structure
//
// ***************************************************************************
void RH_RF95::setModemRegisters(const ModemConfig* config)
	{
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1,config->reg_1d);
    spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2,config->reg_1e);
    spiWrite(RH_RF95_REG_26_MODEM_CONFIG3,config->reg_26);
	}


// ***************************************************************************
//   setModemConfig()
//
//     Set one of the canned FSK Modem configs
//     Returns true if its a valid choice
//
// ***************************************************************************
bool RH_RF95::setModemConfig(ModemConfigChoice index)
	{
    if (index > (signed int)(sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig))) return false;

    ModemConfig cfg;
    memcpy_P(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF95::ModemConfig));
    setModemRegisters(&cfg);

    return true;
	}


// ***************************************************************************
//   setPreambleLength()
//
//     Sets the tx preamble length
//
// ***************************************************************************
void RH_RF95::setPreambleLength(uint16_t bytes)
	{
    spiWrite(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
    spiWrite(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
	}


// ***************************************************************************
//   isChannelActive()
//
//     Detemines if the channel is active
//
// ***************************************************************************
bool RH_RF95::isChannelActive()
	{
    // Set mode RHModeCad
    if (_mode != RHModeCad)
		{
		modeWillChange(RHModeCad);
        spiWrite(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_CAD);
        spiWrite(RH_RF95_REG_40_DIO_MAPPING1, 0x80); // Interrupt on CadDone
        _mode = RHModeCad;
		}

    while (_mode == RHModeCad)

    return _cad;
	}


// ***************************************************************************
//   enableTCXO()
//
//     Enable TCXO
//
// ***************************************************************************
void RH_RF95::enableTCXO(bool on)
	{
    if(on)
		{
		while ((spiRead(RH_RF95_REG_4B_TCXO) & RH_RF95_TCXO_TCXO_INPUT_ON) != RH_RF95_TCXO_TCXO_INPUT_ON)
			{
			sleep();
			spiWrite(RH_RF95_REG_4B_TCXO, (spiRead(RH_RF95_REG_4B_TCXO) | RH_RF95_TCXO_TCXO_INPUT_ON));
			}
		}
    else
		{
		while ((spiRead(RH_RF95_REG_4B_TCXO) & RH_RF95_TCXO_TCXO_INPUT_ON))
			{
			sleep();
			spiWrite(RH_RF95_REG_4B_TCXO, (spiRead(RH_RF95_REG_4B_TCXO) & ~RH_RF95_TCXO_TCXO_INPUT_ON));
			}
		}
	}


// ***************************************************************************
//   frequencyError()
//
//     Determine freq error
//
//     From section 4.1.5 of SX1276/77/78/79 
//     Ferror = FreqError * 2**24 * BW / Fxtal / 500
//
// ***************************************************************************
int RH_RF95::frequencyError()
	{
    int32_t freqerror = 0;

    // Convert 2.5 bytes (5 nibbles, 20 bits) to 32 bit signed int
    // Caution: some C compilers make errors with eg:
    // freqerror = spiRead(RH_RF95_REG_28_FEI_MSB) << 16 so we go more carefully.
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


// ***************************************************************************
//   lastSNR()
//
//     Return the last SNR
//
// ***************************************************************************
int RH_RF95::lastSNR()
	{
    return _lastSNR;
	}


// ***************************************************************************
//   setSpreadingFactor()
//
//     Set the s[reading factor
//
// ***************************************************************************
void RH_RF95::setSpreadingFactor(uint8_t sf)
   {
   if (sf <= 6) 
     sf = RH_RF95_SPREADING_FACTOR_64CPS;
   else if (sf == 7) 
     sf = RH_RF95_SPREADING_FACTOR_128CPS;
   else if (sf == 8) 
     sf = RH_RF95_SPREADING_FACTOR_256CPS;
   else if (sf == 9)
     sf = RH_RF95_SPREADING_FACTOR_512CPS;
   else if (sf == 10)
     sf = RH_RF95_SPREADING_FACTOR_1024CPS;
   else if (sf == 11) 
     sf = RH_RF95_SPREADING_FACTOR_2048CPS;
   else if (sf >= 12)
     sf =  RH_RF95_SPREADING_FACTOR_4096CPS;
 
   // set the new spreading factor
   spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, (spiRead(RH_RF95_REG_1E_MODEM_CONFIG2) & ~RH_RF95_SPREADING_FACTOR) | sf);
   // check if Low data Rate bit should be set or cleared
   setLowDatarate();
   }
 

// ***************************************************************************
//   setSignalBandwidth()
//
//     Set the signal bw
//
// ***************************************************************************
void RH_RF95::setSignalBandwidth(long sbw)
	{
    uint8_t bw; //register bit pattern
 
    if (sbw <= 7800)
		bw = RH_RF95_BW_7_8KHZ;
    else if (sbw <= 10400)
		bw =  RH_RF95_BW_10_4KHZ;
    else if (sbw <= 15600)
		bw = RH_RF95_BW_15_6KHZ ;
    else if (sbw <= 20800)
		bw = RH_RF95_BW_20_8KHZ;
    else if (sbw <= 31250)
		bw = RH_RF95_BW_31_25KHZ;
    else if (sbw <= 41700)
		bw = RH_RF95_BW_41_7KHZ;
    else if (sbw <= 62500)
		bw = RH_RF95_BW_62_5KHZ;
    else if (sbw <= 125000)
		bw = RH_RF95_BW_125KHZ;
    else if (sbw <= 250000)
		bw = RH_RF95_BW_250KHZ;
    else 
		bw =  RH_RF95_BW_500KHZ;
     
    // top 4 bits of reg 1D control bandwidth
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) & ~RH_RF95_BW) | bw);

    // check if low data rate bit should be set or cleared
    setLowDatarate();
	}
 

// ***************************************************************************
//   setCodingRate4()
//
//     Set the coding rate
//
// ***************************************************************************
void RH_RF95::setCodingRate4(uint8_t denominator)
	{
    int cr = RH_RF95_CODING_RATE_4_5;
 
//  if (denominator <= 5)
//	cr = RH_RF95_CODING_RATE_4_5;

    if (denominator == 6)
		cr = RH_RF95_CODING_RATE_4_6;
    else if (denominator == 7)
		cr = RH_RF95_CODING_RATE_4_7;
    else if (denominator >= 8)
		cr = RH_RF95_CODING_RATE_4_8;
 
    // CR is bits 3..1 of RH_RF95_REG_1D_MODEM_CONFIG1
    spiWrite(RH_RF95_REG_1D_MODEM_CONFIG1, (spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) & ~RH_RF95_CODING_RATE) | cr);
	}
 

// ***************************************************************************
//   setLowDatarate()
//
//     Set the low data rate
//
// ***************************************************************************
void RH_RF95::setLowDatarate()
	{
	uint8_t BW;
	uint8_t SF;
	float bandwidth;
	uint8_t current;
	float symbolTime;
	float bw_tab[] = {7800, 10400, 15600, 20800, 31250, 41700, 62500, 125000, 250000, 500000};

    // called after changing bandwidth and/or spreading factor
    //  Semtech modem design guide AN1200.13 says 
    // "To avoid issues surrounding  drift  of  the  crystal  reference  oscillator  due  to  either  temperature  change  
    // or  motion,the  low  data  rate optimization  bit  is  used. Specifically for 125  kHz  bandwidth  and  SF  =  11  and  12,  
    // this  adds  a  small  overhead  to increase robustness to reference frequency variations over the timescale of the LoRa packet."
 
    // read current value for BW and SF
    BW = spiRead(RH_RF95_REG_1D_MODEM_CONFIG1) >> 4;	// bw is in bits 7..4
    SF = spiRead(RH_RF95_REG_1E_MODEM_CONFIG2) >> 4;	// sf is in bits 7..4
   
    // calculate symbol time (see Semtech AN1200.22 section 4)
    bandwidth = bw_tab[BW];
    symbolTime = 1000.0 * pow(2, SF) / bandwidth;	// ms
   
    // the symbolTime for SF 11 BW 125 is 16.384ms. 
    // and, according to this :- 
    // https://www.thethingsnetwork.org/forum/t/a-point-to-note-lora-low-data-rate-optimisation-flag/12007
    // the LDR bit should be set if the Symbol Time is > 16ms
    // So the threshold used here is 16.0ms
 
    // the LDR is bit 3 of RH_RF95_REG_26_MODEM_CONFIG3
	current = spiRead(RH_RF95_REG_26_MODEM_CONFIG3) & ~RH_RF95_LOW_DATA_RATE_OPTIMIZE; // mask off the LDR bit
    
	if (symbolTime > 16.0)
		spiWrite(RH_RF95_REG_26_MODEM_CONFIG3, current | RH_RF95_LOW_DATA_RATE_OPTIMIZE);
    else
		spiWrite(RH_RF95_REG_26_MODEM_CONFIG3, current);
	}
 

// ***************************************************************************
//   setPayloadCRC()
//
//     set payload CRC 
//
// ***************************************************************************
void RH_RF95::setPayloadCRC(bool on)
	{
	uint8_t current;

    // Payload CRC is bit 2 of register 1E
    current = spiRead(RH_RF95_REG_1E_MODEM_CONFIG2) & ~RH_RF95_PAYLOAD_CRC_ON; // mask off the CRC
   
    if (on)
		spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, current | RH_RF95_PAYLOAD_CRC_ON);
    else
		spiWrite(RH_RF95_REG_1E_MODEM_CONFIG2, current);

    _enableCRC = on;
	}
 

// ***************************************************************************
//   getDeviceVersion()
//
//     get device version
//
// ***************************************************************************
uint8_t RH_RF95::getDeviceVersion()
	{
	_deviceVersion = spiRead(RH_RF95_REG_42_VERSION);
	return _deviceVersion;
	}


uint8_t RH_RF95::spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len)
	{
    uint8_t status = 0;

	noInterrupts();

    SPI.beginTransaction(SPISettings(SPIFREQ, SPIORDER,SPIMODE));
    selectSlave();
    status=SPI.transfer(reg | RH_SPI_WRITE_MASK); // Send the start address with the write mask on
    while (len--) SPI.transfer(*src++);
    deselectSlave();
    SPI.endTransaction();

	interrupts();
    return status;
	}

void RH_RF95::selectSlave()
	{
    digitalWrite(_slaveSelectPin, LOW);
	}
    
void RH_RF95::deselectSlave()
	{
    digitalWrite(_slaveSelectPin, HIGH);
	}


void RH_RF95::spiUsingInterrupt(int intNumber)
	{
	SPI.usingInterrupt(intNumber);
	}


// Blocks until a valid message is received or timeout expires
// Return true if there is a message available
// Works correctly even on millis() rollover
bool RH_RF95::waitAvailableTimeout(uint16_t timeout, uint16_t polldelay)
	{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
		{
        if (available()) return true;
		
		if (polldelay) delay(polldelay);
		}

    return false;
	}

bool RH_RF95::waitPacketSent()
{
    while (_mode == RHModeTx);
    return true;
}

bool RH_RF95::waitPacketSent(uint16_t timeout)
	{
    unsigned long starttime = millis();
    while ((millis() - starttime) < timeout)
		{
		// Any previous transmit finished?
        if (_mode != RHModeTx) return true;
		}
    return false;
	}

// Wait until no channel activity detected or timeout
bool RH_RF95::waitCAD()
{
    if (!_cad_timeout)
	return true;

    // Wait for any channel activity to finish or timeout
    // Sophisticated DCF function...
    // DCF : BackoffTime = random() x aSlotTime
    // 100 - 1000 ms
    // 10 sec timeout
    unsigned long t = millis();
    while (isChannelActive())
    {
         if (millis() - t > _cad_timeout) 
	     return false;


    }

    return true;
}

void RH_RF95::setPromiscuous(bool promiscuous)
{
    _promiscuous = promiscuous;
}

void RH_RF95::setThisAddress(uint8_t address)
{
    _thisAddress = address;
}

void RH_RF95::setHeaderTo(uint8_t to)
{
    _txHeaderTo = to;
}

void RH_RF95::setHeaderFrom(uint8_t from)
{
    _txHeaderFrom = from;
}

void RH_RF95::setHeaderId(uint8_t id)
{
    _txHeaderId = id;
}

void RH_RF95::setHeaderFlags(uint8_t set, uint8_t clear)
{
    _txHeaderFlags &= ~clear;
    _txHeaderFlags |= set;
}

uint8_t RH_RF95::headerTo()
{
    return _rxHeaderTo;
}

uint8_t RH_RF95::headerFrom()
{
    return _rxHeaderFrom;
}

uint8_t RH_RF95::headerId()
{
    return _rxHeaderId;
}

uint8_t RH_RF95::headerFlags()
{
    return _rxHeaderFlags;
}

int16_t RH_RF95::lastRssi()
{
    return _lastRssi;
}

RH_RF95::RHMode  RH_RF95::mode()
{
    return _mode;
}

void  RH_RF95::setMode(RHMode mode)
{
    _mode = mode;
}


// Diagnostic help
void RH_RF95::printBuffer(const char* prompt, const uint8_t* buf, uint8_t len)
{
    Serial.println(prompt);
    uint8_t i;
    for (i = 0; i < len; i++)
    {
	if (i % 16 == 15)
	    Serial.println(buf[i], HEX);
	else
	{
	    Serial.print(buf[i], HEX);
	    Serial.print(' ');
	}
    }
    Serial.println("");
}

uint16_t RH_RF95::rxBad()
{
    return _rxBad;
}

uint16_t RH_RF95::rxGood()
{
    return _rxGood;
}

uint16_t RH_RF95::txGood()
{
    return _txGood;
}

void RH_RF95::setCADTimeout(unsigned long cad_timeout)
{
    _cad_timeout = cad_timeout;
}


// ***************************************************************************
//
//   SPI Routines - Arduino hardware API
//   
//   These are independent of the RFM95 stuff
//
// ***************************************************************************

bool RH_RF95::spiInit(void)
	{
    SPI.begin();

	//initialize the slave select
	pinMode(_slaveSelectPin, OUTPUT);
    deselectSlave();
    delay(100);
    return true;
	}


uint8_t RH_RF95::spiRead(uint8_t reg)
	{
    uint8_t val = 0;

	noInterrupts();

    SPI.beginTransaction(SPISettings(SPIFREQ, SPIORDER,SPIMODE));
    selectSlave();
    SPI.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the address with the write mask off
    val=SPI.transfer(0); 					// The written value is ignored, reg value is read
    deselectSlave();
    SPI.endTransaction();

	interrupts();
    return val;
	}


uint8_t RH_RF95::spiWrite(uint8_t reg, uint8_t val)
	{
    uint8_t status = 0;

	noInterrupts();

    SPI.beginTransaction(SPISettings(SPIFREQ, SPIORDER,SPIMODE));
    selectSlave();
    status=SPI.transfer(reg | RH_SPI_WRITE_MASK); 	// Send the address with the write mask on
    SPI.transfer(val); 								// New value follows

    delayMicroseconds(1);
    deselectSlave();

    SPI.endTransaction();

	interrupts();
    return status;
	}


uint8_t RH_RF95::spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len)
	{
    uint8_t status = 0;

	noInterrupts();

    SPI.beginTransaction(SPISettings(SPIFREQ, SPIORDER,SPIMODE));
    selectSlave();
    status = SPI.transfer(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off
    while (len--) *dest++ = SPI.transfer(0);
    deselectSlave();
    SPI.endTransaction();

	interrupts();
    return status;
	}









