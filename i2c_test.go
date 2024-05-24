package rpio

import "fmt"

func Example() {
	err := I2cBegin(I2c1) // pins 7 to 11
	if err != nil {
		panic(err)
	}

	var buf []byte = make([]byte, 1)
	I2cReadRegister(0x04, buf, 1)
	fmt.Printf("Read 0x%02x\n", buf[0])

	I2cEnd(I2c1)
}
