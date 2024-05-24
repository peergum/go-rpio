package rpio

import (
	"errors"
	"log"
	"time"
)

type I2cDev int

// I2C devices.
// Only I2C1 supported for now.
const (
	I2c0 I2cDev = iota
	I2c1        // aux
	I2c2        // aux
)

const (
	controlReg             = iota /*!< BSC Master Control */
	statusReg                     /*!< BSC Master Status */
	dataLengthReg                 /*!< BSC Master Data Length */
	slaveAddressReg               /*!< BSC Master Slave Address */
	dataFifoReg                   /*!< BSC Master Data FIFO */
	clockDividerReg               /*!< BSC Master Clock Divider */
	dataDelayReg                  /*!< BSC Master Data Delay */
	clockStretchTimeoutReg        /*!< BSC Master Clock Stretch Timeout */
)

// control register masks
const (
	controlI2CEnable     = 0x00008000 /*!< I2C Enable, 0 = disabled, 1 = enabled */
	controlInterruptRX   = 0x00000400 /*!< Interrupt on RX */
	controlInterruptTX   = 0x00000200 /*!< Interrupt on TX */
	controlInterruptDone = 0x00000100 /*!< Interrupt on DONE */
	controlStartTransfer = 0x00000080 /*!< Start transfer, 1 = Start a new transfer */
	controlClearFifo1    = 0x00000020 /*!< Clear FIFO Clear */
	controlClearFifo2    = 0x00000010 /*!< Clear FIFO Clear */
	controlRead          = 0x00000001 /*!<	Read transfer */
)

/* Register masks for BSC_S */
const (
	statusClockStretchTimeout = 0x00000200 /*!< Clock stretch timeout */
	statusError               = 0x00000100 /*!< ACK error */
	statusRXFull              = 0x00000080 /*!< RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
	statusTXFull              = 0x00000040 /*!< TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full */
	statusRXContainsData      = 0x00000020 /*!< RXD FIFO contains data */
	statusTXAcceptsData       = 0x00000010 /*!< TXD FIFO can accept data */
	statusRXNeedsRead         = 0x00000008 /*!< RXR FIFO needs reading (full) */
	statusTXNeedsWrite        = 0x00000004 /*!< TXW FIFO needs writing (full) */
	statusTransferDone        = 0x00000002 /*!< Transfer DONE */
	statusTransferActive      = 0x00000001 /*!< Transfer Active */
)

const (
	FIFOSize = 16 /*!< BSC FIFO size */
)

/*
! \brief bcm2835I2CClockDivider

	Specifies the divider used to generate the I2C clock from the system clock.
	Clock divided is based on nominal base clock rate of 250MHz
*/
const (
	i2cClockDivider2500 = 2500 /*!< 2500 = 10us = 100 kHz */
	i2cClockDivider626  = 626  /*!< 622 = 2.504us = 399.3610 kHz */
	i2cClockDivider150  = 150  /*!< 150 = 60ns = 1.666 MHz (default at reset) */
	i2cClockDivider148  = 148  /*!< 148 = 59ns = 1.689 MHz */
)

/*
! \brief bcm2835I2CReasonCodes

	Specifies the reason codes for the bcm2835_i2c_write and bcm2835_i2c_read functions.
*/
const (
	i2cReasonOK                 = 0               /*!< Success */
	i2cErrorNACK                = 1 << (iota - 1) /*!< Received a NACK */
	i2cErrorClockStretchTimeout                   /*!< Received Clock Stretch Timeout */
	i2cErrorData                                  /*!< Not all data is sent / received */
	i2cErrorTimeout                               /*!< Time out occurred during sending */
)

var (
	I2cMapError             = errors.New("I2C registers not mapped correctly - are you root?")
	i2cByteWaitMicroseconds int64
)

// I2cBegin: Sets all pins of given I2C device to I2C mode
//
//	dev\pin | SDA | SCL |
//	I2c0    |   - |   - |
//	I2c1    |   2 |   3 |
//	I2c2    |   - |   - |
//
// It also resets I2C control register.
//
// Note that you should disable I2C interface in raspi-config first!
func I2cBegin(dev I2cDev) error {
	//i2cMem[csReg] = 0 // reset i2c settings to default
	//if i2cMem[csReg] == 0 {
	//	// this should not read only zeroes after reset -> mem map failed
	//	return I2cMapError
	//}

	for _, pin := range getI2cPins(dev) {
		pin.Mode(I2c)
	}

	cdiv := int(i2cMem[clockDividerReg])
	var coreFreq = 250 * 1000000
	if isBCM2711() {
		coreFreq = 550 * 1000000
	}
	i2cByteWaitMicroseconds = int64(float64(cdiv) / float64(coreFreq) * 1000000 * 9)
	log.Printf("cdiv=%d, freq=%d, Microseconds wait per byte: %d", cdiv, coreFreq/cdiv, i2cByteWaitMicroseconds)

	//clearI2cTxRxFifo()
	//ensure we're staying at 100000kHz (default for the pi and pi sugar)
	//setI2cDiv(i2cClockDivider2500)
	return nil
}

// I2cEnd: Sets I2C pins of given device to default (Input) mode. See I2cBegin.
func I2cEnd(dev I2cDev) {
	var pins = getI2cPins(dev)
	for _, pin := range pins {
		pin.Mode(Input)
	}
}

// I2cSpeed: Set (maximal) speed [Hz] of I2C clock.
// Default is 100kHz, and max is 400kHz, but it's
// probably better not to change the default
func I2cSpeed(speed int) {
	coreFreq := 250 * 1000000
	if isBCM2711() {
		coreFreq = 550 * 1000000
	}
	clockDivider := uint32(coreFreq / speed)
	I2cSetClockDivider(clockDivider)
}

// I2cSetClockDivider sets the value for the clock divider
// you should restrain to 100-400kHz, and use setBaudrate instead
func I2cSetClockDivider(clockDivider uint32) {
	i2cMem[clockDividerReg] = clockDivider
}

// I2cSetSlaveAddress sets the slave address to communicate with
func I2cSetSlaveAddress(addr uint32) {
	i2cMem[slaveAddressReg] = addr
}

func I2cSetBaudrate(baudrate int) {
	coreFreq := 250 * 1000000
	if isBCM2711() {
		coreFreq = 550 * 1000000
	}
	clockDivider := (uint32(coreFreq/baudrate) / 2) * 2 // ensure even number
	I2cSetClockDivider(clockDivider)
}

// I2cTransmit takes one or more bytes and send them to slave.
//
// Data received from slave are ignored.
// Use spread operator to send slice of bytes.
func I2cWrite(data ...byte) int {
	// clear FIFO
	i2cSetBits(controlReg, controlClearFifo1, controlClearFifo1)
	// Clear Status
	i2cMem[statusReg] = statusClockStretchTimeout | statusError | statusTransferDone
	// set data length
	i2cMem[dataLengthReg] = uint32(len(data))

	remaining := len(data)
	i := 0
	reason := i2cReasonOK
	failsafe := len(data) * 10000
	timeout := 0

	for remaining > 0 && i < FIFOSize {
		i2cMem[fifoReg] = uint32(data[i])
		i++
		remaining--
	}

	/* Enable device and start transfer */
	i2cMem[controlReg] = controlI2CEnable | controlStartTransfer

	/* Transfer is over when BCM2835_BSC_S_DONE */
	for timeout == 0 && (i2cMem[statusReg]&statusTransferDone) == 0 {
		for timeout == 0 && remaining > 0 && (i2cMem[statusReg]&statusTXAcceptsData) != 0 {
			/* Write to FIFO */
			i2cMem[fifoReg] = uint32(data[i])
			i++
			remaining--
			/* Make sure we don't loop forever! */
			failsafe--
			if failsafe == 0 {
				timeout = 1
				break
			}
		}
		/* Make sure we don't loop forever! */
		failsafe--
		if failsafe == 0 {
			timeout = 1
			break
		}
	}

	if timeout > 0 {
		/* We had a timeout */
		reason = i2cErrorTimeout
	} else if (i2cMem[statusReg] & statusError) != 0 {
		/* Received a NACK */
		reason = i2cErrorNACK
	} else if (i2cMem[statusReg] & statusClockStretchTimeout) != 0 {
		/* Received Clock Stretch Timeout */
		reason = i2cErrorClockStretchTimeout
	} else if remaining > 0 {
		/* Not all data is sent */
		reason = i2cErrorData
	}

	i2cSetBits(controlReg, statusTransferDone, statusTransferDone)

	return reason
}

/* Read an number of bytes from I2C */
func I2cRead(buf []byte, len uint32) int {
	remaining := len
	i := 0
	reason := i2cReasonOK

	/* Clear FIFO */
	i2cSetBits(controlReg, controlClearFifo1, controlClearFifo1)
	/* Clear Status */
	i2cMem[statusReg] = statusClockStretchTimeout | statusError | statusTransferDone
	/* Set Data Length */
	i2cMem[dataLengthReg] = len
	/* Start read */
	i2cMem[controlReg] = controlI2CEnable | controlStartTransfer | controlRead

	/* wait for transfer to complete */
	for (i2cMem[statusReg] & statusTransferDone) == 0 {
		/* we must empty the FIFO as it is populated and not use any delay */
		for remaining > 0 && (i2cMem[statusReg]&statusRXContainsData) != 0 {
			/* Read from FIFO, no barrier */
			buf[i] = byte(i2cMem[fifoReg])
			i++
			remaining--
		}
	}

	/* transfer has finished - grab any remaining stuff in FIFO */
	for remaining > 0 && (i2cMem[statusReg]&statusRXNeedsRead) != 0 {
		/* Read from FIFO, no barrier */
		buf[i] = byte(i2cMem[fifoReg])
		i++
		remaining--
	}

	/* Received a NACK */
	if (i2cMem[statusReg] & statusError) != 0 {
		reason = i2cErrorNACK
	} else if (i2cMem[statusReg] & statusClockStretchTimeout) != 0 {
		/* Received Clock Stretch Timeout */
		reason = i2cErrorClockStretchTimeout
	} else if remaining > 0 {
		/* Not all data is received */
		reason = i2cErrorData
	}

	i2cSetBits(statusReg, statusTransferDone, statusTransferDone)

	return reason
}

func I2cReadRegister(regAddr int, buf []byte, len uint32) int {
	remaining := len
	i := 0
	reason := i2cReasonOK

	/* Clear FIFO */
	i2cSetBits(controlReg, controlClearFifo1, controlClearFifo1)
	/* Clear Status */
	i2cMem[statusReg] = statusClockStretchTimeout | statusError | statusTransferDone
	/* Set Data Length */
	i2cMem[dataLengthReg] = 1
	/* Enable device and start transfer */
	i2cMem[controlReg] = controlI2CEnable
	i2cMem[fifoReg] = uint32(regAddr)
	i2cMem[controlReg] = controlI2CEnable | controlStartTransfer

	/* poll for transfer has started */
	for (i2cMem[statusReg] & statusTransferActive) == 0 {
		/* Linux may cause us to miss entire transfer stage */
		if i2cMem[statusReg]&statusTransferDone != 0 {
			break
		}
	}

	/* Send a repeated start with read bit set in address */
	i2cMem[dataLengthReg] = len
	i2cMem[controlReg] = controlI2CEnable | controlStartTransfer | controlRead

	/* Wait for write to complete and first byte back. */
	time.Sleep(time.Duration(3*i2cByteWaitMicroseconds) * time.Microsecond)

	/* wait for transfer to complete */
	for i2cMem[statusReg]&statusTransferDone == 0 {
		/* we must empty the FIFO as it is populated and not use any delay */
		for remaining > 0 && i2cMem[statusReg]&statusRXNeedsRead != 0 {
			/* Read from FIFO */
			buf[i] = byte(i2cMem[fifoReg])
			i++
			remaining--
		}
	}

	/* transfer has finished - grab any remaining stuff in FIFO */
	for remaining > 0 && (i2cMem[statusReg]&statusRXNeedsRead) != 0 {
		/* Read from FIFO */
		buf[i] = byte(i2cMem[fifoReg])
		i++
		remaining--
	}

	/* Received a NACK */
	if i2cMem[statusReg]&statusError != 0 {
		reason = i2cErrorNACK
	} else if i2cMem[statusReg]&statusClockStretchTimeout != 0 {
		/* Received Clock Stretch Timeout */
		reason = i2cErrorClockStretchTimeout
	} else if remaining > 0 {
		/* Not all data is sent */
		reason = i2cErrorData
	}

	i2cSetBits(controlReg, statusTransferDone, statusTransferDone)

	return reason
}

///* Sending an arbitrary number of bytes before issuing a repeated start
//// (with no prior stop) and reading a response. Some devices require this behavior.
//*/
//uint8_t bcm2835_i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len)
//{
//#ifdef I2C_V1
//volatile uint32_t* dlen = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
//volatile uint32_t* fifo = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
//volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
//volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
//# else
//volatile uint32_t* dlen = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
//volatile uint32_t* fifo = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
//volatile uint32_t* status = bcm2835_bsc1 + BCM2835_BSC_S/4;
//volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
//#endif
//
//uint32_t remaining = cmds_len;
//uint32_t i = 0;
//uint8_t reason = BCM2835_I2C_REASON_OK;
//
///* Clear FIFO */
//bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1, BCM2835_BSC_C_CLEAR_1 );
//
///* Clear Status */
//bcm2835_peri_write(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
//
///* Set Data Length */
//bcm2835_peri_write(dlen, cmds_len);
//
///* pre populate FIFO with max buffer */
//while( remaining && ( i < BCM2835_BSC_FIFO_SIZE ) )
//{
//bcm2835_peri_write_nb(fifo, cmds[i]);
//i++;
//remaining--;
//}
//
///* Enable device and start transfer */
//bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
//
///* poll for transfer has started (way to do repeated start, from BCM2835 datasheet) */
//while ( !( bcm2835_peri_read(status) & BCM2835_BSC_S_TA ) )
//{
///* Linux may cause us to miss entire transfer stage */
//if (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)
//break;
//}
//
//remaining = buf_len;
//i = 0;
//
///* Send a repeated start with read bit set in address */
//bcm2835_peri_write(dlen, buf_len);
//bcm2835_peri_write(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
//
///* Wait for write to complete and first byte back. */
//bcm2835_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));
//
///* wait for transfer to complete */
//while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE))
//{
///* we must empty the FIFO as it is populated and not use any delay */
//while (remaining && bcm2835_peri_read(status) & BCM2835_BSC_S_RXD)
//{
///* Read from FIFO, no barrier */
//buf[i] = bcm2835_peri_read_nb(fifo);
//i++;
//remaining--;
//}
//}
//
///* transfer has finished - grab any remaining stuff in FIFO */
//while (remaining && (bcm2835_peri_read(status) & BCM2835_BSC_S_RXD))
//{
///* Read from FIFO */
//buf[i] = bcm2835_peri_read(fifo);
//i++;
//remaining--;
//}
//
///* Received a NACK */
//if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR)
//{
//reason = BCM2835_I2C_REASON_ERROR_NACK;
//}
//
///* Received Clock Stretch Timeout */ else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT)
//{
//reason = BCM2835_I2C_REASON_ERROR_CLKT;
//}
//
///* Not all data is sent */ else if (remaining)
//{
//reason = BCM2835_I2C_REASON_ERROR_DATA;
//}
//
//bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE, BCM2835_BSC_S_DONE);
//
//return reason;
//}

func i2cSetBits(register int, value uint32, mask uint32) {
	i2cMem[register] = (i2cMem[register] & ^mask) | (i2cMem[register] | value)
}

func getI2cPins(dev I2cDev) []Pin {
	switch dev {
	case I2c0:
		return []Pin{}
	case I2c1:
		return []Pin{2, 3}
	case I2c2:
		return []Pin{}
	default:
		return []Pin{}
	}
}
