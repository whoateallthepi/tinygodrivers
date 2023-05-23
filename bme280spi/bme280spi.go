// Package bme280spi provides a driver for the BME280 digital combined
// humidity and pressure sensor by Bosch.
//
// Datasheet:
// https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf
package bme280spi

import (
	"fmt"
	"machine"
	"time"

	"tinygo.org/x/drivers"
)

// SensorReading is a structure to return a set of pressure/temperature/
// humidity/altitude readings from a sensor. There are three implied decimal places
// ie -1.234 is represented as integer 1234.
type SensorReading struct {
	Pressure    uint32
	Temperature int32
	Humidity    uint32
	//Altitude    int32
}

// Sensor is an interface to a pressure/temperature/humidity/altitude sensor
type Sensor interface {
	Configure() error
	// Callibrate() error
	Read() (*SensorReading, error)
	Reset()
}

type sensorUsage int

const (
	weatherMonitoring sensorUsage = iota + 1
	humiditySensing
	indoorNavigation
	gaming
)
const (
	bufLength             = 10 //
	lengthRegCallibration = 24 // These are the data lengths for two of the
	lengthH2LSB           = 8  // parts of callibration data on the chip
	lengthDataBurst       = 8  // Size of the data slice temp/presure/humidity
)

// DeviceSPI implements the Sensor interface as an SPI connection to a BME280.
// There is// also an I2C version available, see tinygo.org/x/drivers/bme280
type DeviceSPI struct {
	// Chip select pin
	CSB                     machine.Pin
	buf                     [bufLength]byte // length of longest burst read + 1
	Bus                     drivers.SPI
	calibrationCoefficients calibrationCoefficients
	usage                   sensorUsage
	Config                  Config
}

// NewSPI returns a new device driver. The pin and SPI interface are not
// touched, provide a fully configured SPI object and call Configure to start
// using this device.

type calibrationCoefficients struct {
	t1 uint16
	t2 int16
	t3 int16
	p1 uint16
	p2 int16
	p3 int16
	p4 int16
	p5 int16
	p6 int16
	p7 int16
	p8 int16
	p9 int16
	h1 uint8
	h2 int16
	h3 uint8
	h4 int16
	h5 int16
	h6 int8
}

type Oversampling byte
type Mode byte
type FilterCoefficient byte
type Period byte

// Config contains settings for filtering, sampling, and modes of operation
type Config struct {
	Pressure    Oversampling
	Temperature Oversampling
	Humidity    Oversampling
	Period      Period
	Mode        Mode
	IIR         FilterCoefficient
}

// StatusErr is an implementation of error interface to include a Status integer
type StatusErr struct {
	Status  Status
	Message string
}

// Status is used for error reporting - see StatusErr
type Status int

// Error codes
const (
	Wrongchip = iota + 1
	FailedToReadCompensation
	FailedToGetReading
	ErrorCalculatingTemperature
	ErrorCalculatingHumidity
	ErrorCalculatingPressure
	BadValue
)

func (se StatusErr) Error() string {
	return se.Message
}

// NewSPI returns a new device driver. The pin and SPI interface are not
// touched, provide a fully configured SPI object and call Configure to start
// using this device.
// usage selects soem standard configs
// 1 - weatherMonitoring (default)
// 2- 	humiditySensing
// 3 - indoorNavigation
// 4 - gaming
func NewSPI(csb machine.Pin, spi drivers.SPI, usage sensorUsage) *DeviceSPI {
	return &DeviceSPI{
		CSB:   csb,
		Bus:   spi,
		usage: usage,
	}

}

// Configure configures the BME280 for use. It configures the CSB pin and
// configures the BME280. Assumes the SPI interface is running
func (d *DeviceSPI) Configure() error {
	d.CSB.Configure(machine.PinConfig{Mode: machine.PinOutput})
	d.CSB.High()

	id := d.readRegister(0xD0)

	if id != 0x60 && id != 0x58 {
		return fmt.Errorf("unrecognised device. ChipID:%d", id)
	}

	if d.usage == 0 {
		d.usage = 1 // defualt
	}
	switch d.usage {
	case indoorNavigation:
		d.Config = Config{
			Mode:        ModeNormal,
			Period:      Period0_5ms,
			Temperature: Sampling2X,
			Humidity:    Sampling1X,
			Pressure:    Sampling16X,
			IIR:         Coeff16,
		}

	case weatherMonitoring:
		d.Config = Config{
			Mode:        ModeForced,
			Period:      Period0_5ms,
			Temperature: Sampling1X,
			Humidity:    Sampling1X,
			Pressure:    Sampling1X,
			IIR:         Coeff0,
		}
	case humiditySensing:
		d.Config = Config{
			Mode:        ModeForced,
			Period:      Period0_5ms,
			Temperature: Sampling1X,
			Humidity:    Sampling1X,
			Pressure:    SamplingOff,
			IIR:         Coeff0,
		}
	case gaming:
		d.Config = Config{
			Mode:        ModeNormal,
			Period:      Period0_5ms,
			Temperature: Sampling1X,
			Humidity:    SamplingOff,
			Pressure:    Sampling4X,
			IIR:         Coeff16,
		}
	default:
		return fmt.Errorf(" unrecognised sensorMode: %d\\n", d.usage)
	}

	data, err := d.readRegisters(REG_CALIBRATION, lengthRegCallibration)

	if err != nil {
		return StatusErr{
			Status:  FailedToReadCompensation,
			Message: fmt.Sprintf("failed to read calibration  registers"),
		}
	}

	h1, err := d.readRegisters(REG_CALIBRATION_H1, 1)
	if err != nil {
		return StatusErr{
			Status:  FailedToReadCompensation,
			Message: fmt.Sprintf("failed to read callibration H1 register"),
		}
	}

	h2lsb, err := d.readRegisters(REG_CALIBRATION_H2LSB, lengthH2LSB)
	//fmt.Printf("h2lsb: %x", h2lsb)

	if err != nil {
		return StatusErr{
			Status:  FailedToReadCompensation,
			Message: fmt.Sprintf("failed to read callibration H2LSB register"),
		}
	}
	d.calibrationCoefficients.t1 = readUintLE(data[0], data[1])
	d.calibrationCoefficients.t2 = readIntLE(data[2], data[3])
	d.calibrationCoefficients.t3 = readIntLE(data[4], data[5])
	d.calibrationCoefficients.p1 = readUintLE(data[6], data[7])
	d.calibrationCoefficients.p2 = readIntLE(data[8], data[9])
	d.calibrationCoefficients.p3 = readIntLE(data[10], data[11])
	d.calibrationCoefficients.p4 = readIntLE(data[12], data[13])
	d.calibrationCoefficients.p5 = readIntLE(data[14], data[15])
	d.calibrationCoefficients.p6 = readIntLE(data[16], data[17])
	d.calibrationCoefficients.p7 = readIntLE(data[18], data[19])
	d.calibrationCoefficients.p8 = readIntLE(data[20], data[21])
	d.calibrationCoefficients.p9 = readIntLE(data[22], data[23])

	d.calibrationCoefficients.h1 = h1[0]
	d.calibrationCoefficients.h2 = readIntLE(h2lsb[0], h2lsb[1])
	d.calibrationCoefficients.h3 = h2lsb[2]
	d.calibrationCoefficients.h6 = int8(h2lsb[7])
	d.calibrationCoefficients.h4 = 0 + (int16(h2lsb[3]) << 4) | (int16(h2lsb[4] & 0x0F))
	d.calibrationCoefficients.h5 = 0 + (int16(h2lsb[5]) >> 4) | (int16(h2lsb[6]) << 4)

	d.reset() // Find out why!

	configMask := []byte{byte(d.Config.Period<<5) | byte(d.Config.IIR<<2)}[0] // maybe tidy this up

	//[]byte{byte(d.Config.Period<<5) | byte(d.Config.IIR<<2)})
	d.writeRegister(CTRL_CONFIG, configMask)

	// d.writeRegister(CTRL_CONFIG, (byte(d.Config.Period<<5) | byte(d.Config.IIR<<2)))
	d.writeRegister(CTRL_HUMIDITY_ADDR, byte(d.Config.Humidity))

	// Normal mode, start measuring now
	if d.Config.Mode == ModeNormal {
		modeMask := []byte{
			byte(d.Config.Temperature<<5) |
				byte(d.Config.Pressure<<2) |
				byte(d.Config.Mode)}[0]

		d.writeRegister(CTRL_MEAS_ADDR, modeMask)
	}

	//fmt.Print(configMask)
	//d.writeRegister(0xF2, 0x1)  // Humidity oversampling
	//d.writeRegister(0xF4, 0x27) // Run  mode normal & other oversamplings

	return nil
}

// Reset is a method to reset the device
func (d *DeviceSPI) Reset() {
	d.writeRegister(CMD_RESET, 0xB6)
}

// Read is a method to read the sensor and return a SensorReading
// data structure
func (d *DeviceSPI) Read() (*SensorReading, error) {

	data, err := d.readData()
	if err != nil {
		return nil, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("failed read temp/pressure/humidity"),
		}
	}
	t, tfine, err := d.calculateTemp(data)

	if err != nil {
		return nil, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("internal error %s", err),
		}
	}

	h, err := d.calculateHumidity(data, tfine)

	if err != nil {
		return nil, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("internal error %s", err),
		}
	}

	p, err := d.calculatePressure(data, tfine)

	if err != nil {
		return nil, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("internal error %s", err),
		}
	}

	sr := SensorReading{Temperature: t,
		Pressure: p,
		Humidity: h}

	return &sr, nil
}

// readRegister reads from a single BMI160 register. It should only be used for
// single register reads, not for reading multiple registers at once.
func (d *DeviceSPI) readRegister(address uint8) uint8 {
	// I don't know why but it appears necessary to sleep for a bit here.
	time.Sleep(time.Millisecond)

	data := d.buf[:2]
	data[0] = 0x80 | address
	data[1] = 0
	d.CSB.Low()
	d.Bus.Tx(data, data)
	d.CSB.High()
	return data[1]
}

// readRegisters reads multiple registers
func (d *DeviceSPI) readRegisters(address uint8, length uint8) ([]byte, error) {

	data := make([]byte, length+1) //data buffer
	data[0] = 0x80 | address
	d.CSB.Low()
	d.Bus.Tx(data, data)
	d.CSB.High()
	return data[1:], nil

}

// writeRegister writes a single byte register. It should only be used
// for writing to a single register.
func (d *DeviceSPI) writeRegister(address, data uint8) {
	// I don't know why but it appears necessary to sleep for a bit here.
	time.Sleep(time.Millisecond)

	buf := d.buf[:2]
	buf[0] = address & 0x7F // remove read bit - this is a write
	buf[1] = data

	d.CSB.Low()
	d.Bus.Tx(buf, buf)
	d.CSB.High()
}

// readData does a burst read from 0xF7 to 0xF0 according to the datasheet
// resulting in an slice with 8 bytes 0-2 = pressure / 3-5 = temperature / 6-7 = humidity
func (d *DeviceSPI) readData() ([]byte, error) {
	if d.Config.Mode == ModeForced {
		// Write the CTRL_MEAS register to trigger a measurement
		modeMask := []byte{
			byte(d.Config.Temperature<<5) |
				byte(d.Config.Pressure<<2) |
				byte(d.Config.Mode)}[0]

		//fmt.Print(modeMask)
		d.writeRegister(CTRL_MEAS_ADDR, modeMask)

		time.Sleep(d.measurementDelay())
	}

	time.Sleep(time.Second)

	db := d.buf[:(lengthDataBurst + 1)]

	clearBuffer(db)

	db[0] = REG_PRESSURE | 0x80

	d.CSB.Low()
	time.Sleep(10 * time.Millisecond)

	err := d.Bus.Tx(db, db)

	d.CSB.High()

	if err != nil {
		return nil, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("failed to get temp/pressure/humidity data"),
		}
	}
	return db[1:], nil // first byte will be the 0xFF corresponding to the register address
}

// reset the device
func (d *DeviceSPI) reset() {
	d.writeRegister(CMD_RESET, 0xB6)
}

// measurementDelay is used in forced mode to wait until a measurement is complete.
func (d *DeviceSPI) measurementDelay() time.Duration {
	const MeasOffset = 1250
	const MeasDur = 2300
	const HumMeasOffset = 575
	const MeasScalingFactor = 1000

	// delay is based on over-sampling rate - this table converts from
	// setting to number samples
	sampleRateConv := []int{0, 1, 2, 4, 8, 16}

	tempOsr := 16
	if d.Config.Temperature <= Sampling16X {
		tempOsr = sampleRateConv[d.Config.Temperature]
	}

	presOsr := 16
	if d.Config.Temperature <= Sampling16X {
		presOsr = sampleRateConv[d.Config.Pressure]
	}

	humOsr := 16
	if d.Config.Temperature <= Sampling16X {
		humOsr = sampleRateConv[d.Config.Humidity]
	}

	max_delay := ((MeasOffset + (MeasDur * tempOsr) +
		((MeasDur * presOsr) + HumMeasOffset) +
		((MeasDur * humOsr) + HumMeasOffset)) / MeasScalingFactor)

	return time.Duration(max_delay) * time.Millisecond
}

// calculateTemp uses the data slice and applies calibrations values on it to
// convert the value to an integer of one hundreths of degrees
// eg 12.75 C is 1275
// It also calculates the variable tFine which is used by the pressure and humidity calculation
func (d *DeviceSPI) calculateTemp(data []byte) (int32, int32, error) {

	rawTemp := convert3Bytes(data[3], data[4], data[5])

	var1 := (((rawTemp >> 3) - (int32(d.calibrationCoefficients.t1) << 1)) * int32(d.calibrationCoefficients.t2)) >> 11
	var2 := (((((rawTemp >> 4) - int32(d.calibrationCoefficients.t1)) * ((rawTemp >> 4) - int32(d.calibrationCoefficients.t1))) >> 12) * int32(d.calibrationCoefficients.t3)) >> 14

	tFine := var1 + var2
	T := (tFine*5 + 128) >> 8
	return T, tFine, nil
}

// calculateHumidity takes the databurst and calculates the humidity
// Returns an integer in hundreths of a percentage point
// eg 75.32% = 7532
func (d *DeviceSPI) calculateHumidity(data []byte, tFine int32) (uint32, error) {

	rawHumidity := convert2Bytes(data[6], data[7])
	//fmt.Printf("Raw Humidity: %d\n tFine: %d\n", rawHumidity, tFine)
	h := float32(tFine) - 76800

	if h == 0 {
		return 0, StatusErr{
			Status:  ErrorCalculatingHumidity,
			Message: fmt.Sprintf("failed to calculate humidity - h == 0"),
		}
	}

	var1 := float32(rawHumidity) - (float32(d.calibrationCoefficients.h4)*64.0 +
		(float32(d.calibrationCoefficients.h5) / 16384.0 * h))

	var2 := float32(d.calibrationCoefficients.h2) / 65536.0 *
		(1.0 + float32(d.calibrationCoefficients.h6)/67108864.0*h*
			(1.0+float32(d.calibrationCoefficients.h3)/67108864.0*h))

	h = var1 * var2
	h = h * (1 - float32(d.calibrationCoefficients.h1)*h/524288)

	//fmt.Printf("h: %02.2f\n", h)
	return uint32(100 * h), nil

}

// calculatePressure uses the data slice and applies calibrations values on it
// to convert the value to Pascals. Standard air pressure is 101325 Pa.
// For millibars (or the more modern hPa) divide by 100.
func (d *DeviceSPI) calculatePressure(data []byte, tFine int32) (uint32, error) {

	rawPressure := convert3Bytes(data[0], data[1], data[2])

	var1 := int64(tFine) - 128000
	var2 := var1 * var1 * int64(d.calibrationCoefficients.p6)
	var2 = var2 + ((var1 * int64(d.calibrationCoefficients.p5)) << 17)
	var2 = var2 + (int64(d.calibrationCoefficients.p4) << 35)
	var1 = ((var1 * var1 * int64(d.calibrationCoefficients.p3)) >> 8) + ((var1 * int64(d.calibrationCoefficients.p2)) << 12)
	var1 = ((int64(1) << 47) + var1) * int64(d.calibrationCoefficients.p1) >> 33

	if var1 == 0 {
		return 0, StatusErr{
			Status:  FailedToGetReading,
			Message: fmt.Sprintf("failed to calculate pressure - var1 == 0"),
		} // avoid exception caused by division by zero
	}
	p := int64(1048576 - rawPressure)
	p = (((p << 31) - var2) * 3125) / var1
	var1 = (int64(d.calibrationCoefficients.p9) * (p >> 13) * (p >> 13)) >> 25
	var2 = (int64(d.calibrationCoefficients.p8) * p) >> 19

	p = ((p + var1 + var2) >> 8) + (int64(d.calibrationCoefficients.p7) << 4)
	p = (p / 256)
	return uint32(p), nil
}

func clearBuffer(buffer []byte) {
	for i := range buffer {
		buffer[i] = 0x00
	}
}

// ============================= Utilities =====================================
// convert2Bytes converts two bytes to int32
func convert2Bytes(msb byte, lsb byte) int32 {
	return int32(readUint(msb, lsb))
}

// convert3Bytes converts three bytes to int32
func convert3Bytes(msb byte, b1 byte, lsb byte) int32 {
	return int32(((((uint32(msb) << 8) | uint32(b1)) << 8) | uint32(lsb)) >> 4)
}

// readUint converts two bytes to uint16
func readUint(msb byte, lsb byte) uint16 {
	return (uint16(msb) << 8) | uint16(lsb)
}

// readUintLE converts two little endian bytes to uint16
func readUintLE(msb byte, lsb byte) uint16 {
	temp := readUint(msb, lsb)
	return (temp >> 8) | (temp << 8)
}

// readIntLE converts two little endian bytes to int16
func readIntLE(msb byte, lsb byte) int16 {
	return int16(readUintLE(msb, lsb))
}
