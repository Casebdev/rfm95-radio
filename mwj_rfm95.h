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
// ***************************************************************************

#include "Arduino.h"
#include <avr/interrupt.h >
#include <SPI.h>

#ifndef MWJ_RFM95_h
#define MWJ_RFM95_h

//SPI interface configuration
#define SPIORDER	MSBFIRST
#define SPIFREQ		2000000
#define SPIMODE		SPI_MODE0

//interrupt pin definitions
#define RH_INVALID_PIN 0xff

//SPI addressing
#define RH_BROADCAST_ADDRESS 0xff
#define RH_SPI_WRITE_MASK 0x80

// Max number of octets the LORA Rx/Tx FIFO can hold
#define RH_RF95_FIFO_SIZE 255

// This is the maximum number of bytes that can be carried by the LORA.
// We use some for headers, keeping fewer for user messages
#define RH_RF95_MAX_PAYLOAD_LEN RH_RF95_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the LORA's payload
#define RH_RF95_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data and 2 bytes of FCS
#define RH_RF95_MAX_MESSAGE_LEN (RH_RF95_MAX_PAYLOAD_LEN - RH_RF95_HEADER_LEN)

// The crystal oscillator frequency of the module
#define RH_RF95_FXOSC 32000000.0

// The Frequency Synthesizer step = RH_RF95_FXOSC / 2^^19
#define RH_RF95_FSTEP  (RH_RF95_FXOSC / 524288)

// Register names (LoRa Mode, from table 85)
#define RH_RF95_REG_00_FIFO                                0x00
#define RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_REG_02_RESERVED                            0x02
#define RH_RF95_REG_03_RESERVED                            0x03
#define RH_RF95_REG_04_RESERVED                            0x04
#define RH_RF95_REG_05_RESERVED                            0x05
#define RH_RF95_REG_06_FRF_MSB                             0x06
#define RH_RF95_REG_07_FRF_MID                             0x07
#define RH_RF95_REG_08_FRF_LSB                             0x08
#define RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_REG_0D_FIFO_ADDR_PTR                       0x0d
#define RH_RF95_REG_0E_FIFO_TX_BASE_ADDR                   0x0e
#define RH_RF95_REG_0F_FIFO_RX_BASE_ADDR                   0x0f
#define RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR                0x10
#define RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_REG_13_RX_NB_BYTES                         0x13
#define RH_RF95_REG_14_RX_HEADER_CNT_VALUE_MSB             0x14
#define RH_RF95_REG_15_RX_HEADER_CNT_VALUE_LSB             0x15
#define RH_RF95_REG_16_RX_PACKET_CNT_VALUE_MSB             0x16
#define RH_RF95_REG_17_RX_PACKET_CNT_VALUE_LSB             0x17
#define RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_REG_19_PKT_SNR_VALUE                       0x19
#define RH_RF95_REG_1A_PKT_RSSI_VALUE                      0x1a
#define RH_RF95_REG_1B_RSSI_VALUE                          0x1b
#define RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_REG_1F_SYMB_TIMEOUT_LSB                    0x1f
#define RH_RF95_REG_20_PREAMBLE_MSB                        0x20
#define RH_RF95_REG_21_PREAMBLE_LSB                        0x21
#define RH_RF95_REG_22_PAYLOAD_LENGTH                      0x22
#define RH_RF95_REG_23_MAX_PAYLOAD_LENGTH                  0x23
#define RH_RF95_REG_24_HOP_PERIOD                          0x24
#define RH_RF95_REG_25_FIFO_RX_BYTE_ADDR                   0x25
#define RH_RF95_REG_26_MODEM_CONFIG3                       0x26

#define RH_RF95_REG_27_PPM_CORRECTION                      0x27
#define RH_RF95_REG_28_FEI_MSB                             0x28
#define RH_RF95_REG_29_FEI_MID                             0x29
#define RH_RF95_REG_2A_FEI_LSB                             0x2a
#define RH_RF95_REG_2C_RSSI_WIDEBAND                       0x2c
#define RH_RF95_REG_31_DETECT_OPTIMIZE                     0x31
#define RH_RF95_REG_33_INVERT_IQ                           0x33
#define RH_RF95_REG_37_DETECTION_THRESHOLD                 0x37
#define RH_RF95_REG_39_SYNC_WORD                           0x39

#define RH_RF95_REG_40_DIO_MAPPING1                        0x40
#define RH_RF95_REG_41_DIO_MAPPING2                        0x41
#define RH_RF95_REG_42_VERSION                             0x42

#define RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_REG_5B_FORMER_TEMP                         0x5b
#define RH_RF95_REG_61_AGC_REF                             0x61
#define RH_RF95_REG_62_AGC_THRESH1                         0x62
#define RH_RF95_REG_63_AGC_THRESH2                         0x63
#define RH_RF95_REG_64_AGC_THRESH3                         0x64

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE                       0x80
#define RH_RF95_ACCESS_SHARED_REG                     0x40
#define RH_RF95_LOW_FREQUENCY_MODE                    0x08
#define RH_RF95_MODE                                  0x07
#define RH_RF95_MODE_SLEEP                            0x00
#define RH_RF95_MODE_STDBY                            0x01
#define RH_RF95_MODE_FSTX                             0x02
#define RH_RF95_MODE_TX                               0x03
#define RH_RF95_MODE_FSRX                             0x04
#define RH_RF95_MODE_RXCONTINUOUS                     0x05
#define RH_RF95_MODE_RXSINGLE                         0x06
#define RH_RF95_MODE_CAD                              0x07

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT                             0x80
#define RH_RF95_MAX_POWER                             0x70
#define RH_RF95_OUTPUT_POWER                          0x0f

// RH_RF95_REG_0A_PA_RAMP                             0x0a
#define RH_RF95_LOW_PN_TX_PLL_OFF                     0x10
#define RH_RF95_PA_RAMP                               0x0f
#define RH_RF95_PA_RAMP_3_4MS                         0x00
#define RH_RF95_PA_RAMP_2MS                           0x01
#define RH_RF95_PA_RAMP_1MS                           0x02
#define RH_RF95_PA_RAMP_500US                         0x03
#define RH_RF95_PA_RAMP_250US                         0x04
#define RH_RF95_PA_RAMP_125US                         0x05
#define RH_RF95_PA_RAMP_100US                         0x06
#define RH_RF95_PA_RAMP_62US                          0x07
#define RH_RF95_PA_RAMP_50US                          0x08
#define RH_RF95_PA_RAMP_40US                          0x09
#define RH_RF95_PA_RAMP_31US                          0x0a
#define RH_RF95_PA_RAMP_25US                          0x0b
#define RH_RF95_PA_RAMP_20US                          0x0c
#define RH_RF95_PA_RAMP_15US                          0x0d
#define RH_RF95_PA_RAMP_12US                          0x0e
#define RH_RF95_PA_RAMP_10US                          0x0f

// RH_RF95_REG_0B_OCP                                 0x0b
#define RH_RF95_OCP_ON                                0x20
#define RH_RF95_OCP_TRIM                              0x1f

// RH_RF95_REG_0C_LNA                                 0x0c
#define RH_RF95_LNA_GAIN                              0xe0
#define RH_RF95_LNA_GAIN_G1                           0x20
#define RH_RF95_LNA_GAIN_G2                           0x40
#define RH_RF95_LNA_GAIN_G3                           0x60                
#define RH_RF95_LNA_GAIN_G4                           0x80
#define RH_RF95_LNA_GAIN_G5                           0xa0
#define RH_RF95_LNA_GAIN_G6                           0xc0
#define RH_RF95_LNA_BOOST_LF                          0x18
#define RH_RF95_LNA_BOOST_LF_DEFAULT                  0x00
#define RH_RF95_LNA_BOOST_HF                          0x03
#define RH_RF95_LNA_BOOST_HF_DEFAULT                  0x00
#define RH_RF95_LNA_BOOST_HF_150PC                    0x03

// RH_RF95_REG_11_IRQ_FLAGS_MASK                      0x11
#define RH_RF95_RX_TIMEOUT_MASK                       0x80
#define RH_RF95_RX_DONE_MASK                          0x40
#define RH_RF95_PAYLOAD_CRC_ERROR_MASK                0x20
#define RH_RF95_VALID_HEADER_MASK                     0x10
#define RH_RF95_TX_DONE_MASK                          0x08
#define RH_RF95_CAD_DONE_MASK                         0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL_MASK              0x02
#define RH_RF95_CAD_DETECTED_MASK                     0x01

// RH_RF95_REG_12_IRQ_FLAGS                           0x12
#define RH_RF95_RX_TIMEOUT                            0x80
#define RH_RF95_RX_DONE                               0x40
#define RH_RF95_PAYLOAD_CRC_ERROR                     0x20
#define RH_RF95_VALID_HEADER                          0x10
#define RH_RF95_TX_DONE                               0x08
#define RH_RF95_CAD_DONE                              0x04
#define RH_RF95_FHSS_CHANGE_CHANNEL                   0x02
#define RH_RF95_CAD_DETECTED                          0x01

// RH_RF95_REG_18_MODEM_STAT                          0x18
#define RH_RF95_RX_CODING_RATE                        0xe0
#define RH_RF95_MODEM_STATUS_CLEAR                    0x10
#define RH_RF95_MODEM_STATUS_HEADER_INFO_VALID        0x08
#define RH_RF95_MODEM_STATUS_RX_ONGOING               0x04
#define RH_RF95_MODEM_STATUS_SIGNAL_SYNCHRONIZED      0x02
#define RH_RF95_MODEM_STATUS_SIGNAL_DETECTED          0x01

// RH_RF95_REG_1C_HOP_CHANNEL                         0x1c
#define RH_RF95_PLL_TIMEOUT                           0x80
#define RH_RF95_RX_PAYLOAD_CRC_IS_ON                  0x40
#define RH_RF95_FHSS_PRESENT_CHANNEL                  0x3f

// RH_RF95_REG_1D_MODEM_CONFIG1                       0x1d
#define RH_RF95_BW                                    0xf0

#define RH_RF95_BW_7_8KHZ                             0x00
#define RH_RF95_BW_10_4KHZ                            0x10
#define RH_RF95_BW_15_6KHZ                            0x20
#define RH_RF95_BW_20_8KHZ                            0x30
#define RH_RF95_BW_31_25KHZ                           0x40
#define RH_RF95_BW_41_7KHZ                            0x50
#define RH_RF95_BW_62_5KHZ                            0x60
#define RH_RF95_BW_125KHZ                             0x70
#define RH_RF95_BW_250KHZ                             0x80
#define RH_RF95_BW_500KHZ                             0x90
#define RH_RF95_CODING_RATE                           0x0e
#define RH_RF95_CODING_RATE_4_5                       0x02
#define RH_RF95_CODING_RATE_4_6                       0x04
#define RH_RF95_CODING_RATE_4_7                       0x06
#define RH_RF95_CODING_RATE_4_8                       0x08
#define RH_RF95_IMPLICIT_HEADER_MODE_ON               0x01

// RH_RF95_REG_1E_MODEM_CONFIG2                       0x1e
#define RH_RF95_SPREADING_FACTOR                      0xf0
#define RH_RF95_SPREADING_FACTOR_64CPS                0x60
#define RH_RF95_SPREADING_FACTOR_128CPS               0x70
#define RH_RF95_SPREADING_FACTOR_256CPS               0x80
#define RH_RF95_SPREADING_FACTOR_512CPS               0x90
#define RH_RF95_SPREADING_FACTOR_1024CPS              0xa0
#define RH_RF95_SPREADING_FACTOR_2048CPS              0xb0
#define RH_RF95_SPREADING_FACTOR_4096CPS              0xc0
#define RH_RF95_TX_CONTINUOUS_MODE                    0x08

#define RH_RF95_PAYLOAD_CRC_ON                        0x04
#define RH_RF95_SYM_TIMEOUT_MSB                       0x03

// RH_RF95_REG_26_MODEM_CONFIG3
#define RH_RF95_MOBILE_NODE                           0x08 // HopeRF term
#define RH_RF95_LOW_DATA_RATE_OPTIMIZE                0x08 // Semtechs term
#define RH_RF95_AGC_AUTO_ON                           0x04

// RH_RF95_REG_4B_TCXO                                0x4b
#define RH_RF95_TCXO_TCXO_INPUT_ON                    0x10

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE                        0x04
#define RH_RF95_PA_DAC_ENABLE                         0x07


// ********************************************************************
//   class: RH_RF95
//
//   Interface to the RFM95 Radio
//	 Assumes the use of hardware SPI
//
// ********************************************************************
class RH_RF95
{

public:

    typedef enum
		{
		RHModeInitialising = 0, ///< Transport is initialising. Initial default value until init() is called..
		RHModeSleep,            ///< Transport hardware is in low power sleep mode (if supported)
		RHModeIdle,             ///< Transport is idle.
		RHModeTx,               ///< Transport is in the process of transmitting a message.
		RHModeRx,               ///< Transport is in the process of receiving a message.
		RHModeCad               ///< Transport is in the process of detecting channel activity (if supported)
		} RHMode;

    typedef struct
		{
		uint8_t reg_1d;   ///< Value for register RH_RF95_REG_1D_MODEM_CONFIG1
		uint8_t reg_1e;   ///< Value for register RH_RF95_REG_1E_MODEM_CONFIG2
		uint8_t reg_26;   ///< Value for register RH_RF95_REG_26_MODEM_CONFIG3
		} ModemConfig;

    // predefined model configurations
    typedef enum
		{
		Bw125Cr45Sf128 = 0,	   // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
		Bw500Cr45Sf128,	       // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
		Bw31_25Cr48Sf512,	   // Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
		Bw125Cr48Sf4096,       // Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
		Bw125Cr45Sf2048,       // Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
		} ModemConfigChoice;

    //constructors
    RH_RF95(uint8_t slaveSelectPin = 10, uint8_t interruptPin = 2);

    //functions that define the interface to the RF95
    bool init();
    bool printRegisters();
    void setModemRegisters(const ModemConfig* config);
    virtual bool available();
    virtual bool recv(uint8_t* buf, uint8_t* len);			//get received data
    virtual bool send(const uint8_t* data, uint8_t len);	//send data
    void setPreambleLength(uint16_t bytes);					//set device preamble length
    virtual uint8_t maxMessageLength();						//set max message length
    bool setFrequency(float centre);						//set device frequency
    void setModeIdle();										//put device into idle mode
    void setModeRx();										//put device into Rx mode
    void setModeTx();										//put device into Tx mode
    void setTxPower(int8_t power, bool useRFO = false);		//set tx power
    virtual bool sleep();									//put device into sleep mode		
    virtual bool    isChannelActive();						//determine if channel is active
    void enableTCXO(bool on = true);						//enable TCXO model
    int frequencyError();									//get last frequency error
    int lastSNR();											//get last SNR
    void setSpreadingFactor(uint8_t sf);					//set radio spreading factor
    void setSignalBandwidth(long sbw);						//set signal bandwidth
    void setCodingRate4(uint8_t denominator);				//set the coding rate
    void setLowDatarate();									//set the low data rate
    void setPayloadCRC(bool on);							//enable payload CRC
    uint8_t getDeviceVersion();								//get device version information

	bool waitAvailableTimeout(uint16_t timeout, uint16_t polldelay);
	bool waitPacketSent();
	bool waitPacketSent(uint16_t timeout);
	bool waitCAD();
	void setPromiscuous(bool promiscuous);
	void setThisAddress(uint8_t address);
	void setHeaderTo(uint8_t to);
	void setHeaderFrom(uint8_t from);
	void setHeaderId(uint8_t id);
	void setHeaderFlags(uint8_t set, uint8_t clear);
	uint8_t headerTo();
	uint8_t headerFrom();
	uint8_t headerId();
	uint8_t headerFlags();
	int16_t lastRssi();
	void printBuffer(const char* prompt, const uint8_t* buf, uint8_t len);
	bool setModemConfig(ModemConfigChoice index);
	uint16_t rxBad();
	uint16_t rxGood();
	uint16_t txGood();
	void setCADTimeout(unsigned long cad_timeout);

protected:
	static void isr0();
    void handleInterrupt();					//interrupt handler 
    void validateRxBuf();					//determinen if message is for this node
    void clearRxBuf();						//clear rx buffer

	virtual bool modeWillChange(RHMode) {return true;}

    bool _useRFO;							//False - use PA_BOOST tx out pin, true - use RFO tx output pin
    
private:
	static RH_RF95* _deviceForInterrupt;

	bool spiInit(void);
	uint8_t spiRead(uint8_t reg);
	uint8_t spiWrite(uint8_t reg, uint8_t val);
	uint8_t spiBurstRead(uint8_t reg, uint8_t* dest, uint8_t len);
	uint8_t spiBurstWrite(uint8_t reg, const uint8_t* src, uint8_t len);
	void spiUsingInterrupt(int intNumber);
	void selectSlave();
	void deselectSlave();

    uint8_t _interruptPin;					// The configured interrupt pin connected to this instance
    uint8_t _slaveSelectPin;				// The configured slave select pin connected to this instance
    volatile uint8_t _bufLen;				// Number of octets in the buffer
    uint8_t	_buf[RH_RF95_MAX_PAYLOAD_LEN];	// The receiver/transmitter buffer
    volatile bool _rxBufValid;				//message available flag
    bool _usingHFport;						// The receiver/transmitter buffer
    int8_t _lastSNR;						// Last measured SNR, dB
    bool _enableCRC;						//If true, sends CRCs in every packet and requires a valid CRC in every received packet
    uint8_t _deviceVersion = 0x00;			//device ID

	volatile RHMode _mode;					//device mode
    uint8_t _thisAddress;					// This node id
    bool _promiscuous;						// Whether the transport is in promiscuous mode
    volatile uint8_t _rxHeaderTo;			// TO header in the last received mesasge
	volatile uint8_t _rxHeaderFrom;			// FROM header in the last received mesasge
    volatile uint8_t _rxHeaderId;			// ID header in the last received mesasge
    volatile uint8_t _rxHeaderFlags;		// FLAGS header in the last received mesasge
    uint8_t _txHeaderTo;					// TO header to send in all messages
    uint8_t _txHeaderFrom;					// FROM header to send in all messages
    uint8_t _txHeaderId;					// ID header to send in all messages
    uint8_t _txHeaderFlags;					// FLAGS header to send in all messages
    volatile int16_t _lastRssi;				// The value of the last received RSSI value, in some transport specific units
    volatile uint16_t _rxBad;				// Count of the number of bad messages (eg bad checksum etc) received
    volatile uint16_t _rxGood;				// Count of the number of successfully transmitted messaged
    volatile uint16_t _txGood;				// Count of the number of bad messages (correct checksum etc) received
    volatile bool _cad;						// Channel activity detected 
    unsigned int _cad_timeout;				// Channel activity timeout in ms

	RHMode  mode();
	void  setMode(RHMode mode);
};

#endif