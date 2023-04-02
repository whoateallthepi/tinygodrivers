// Package ds3231 provides a driver for the DS3231 RTC
// forked from tinygo.org/x/drivers/ds3231 to add alarm functions
// Datasheet: https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
//
// The ds3231 supports two alarms - alarm 1 and alarm 2. Alarm one supports
// alarms every second, every minute (ie seconds match), every hour
// (minutes and seconds match), every day (hours minutes and seconds match).
//
// Alarm 2 can trigger every minute - when seconds are 00, when minutes match,
// and when hours and minutes match.
//
// Both alarms also support trigerring when date, hours minutes and seconds match or
// day, hours minutes and seconds match. This dirver does not currently
// implement that feature.
//
// This driver doesn't implement all the capabilities of the ds3231 chip, but it supports a
// basic set of functions that are common to many clocks (via the Clocker interface). These are:
//
//  Configure() error - prepares the clock, checks it is running
//
//	Read() (time.Time, error) - returns the time
//
//	Set(time.Time) error - sets the time
//
//	IsTimeValid() bool - checks if clock is set
//
//	SetAlarm(time.Time, Alarm, AlarmPeriod) error - sets one of the alarms to the time
//													and and frequencey chosen,switches the alarm on
//
//	SilenceAlarm(Alarm) error - After an alarm triggers, resets it so it can trigger next time
//
//	TurnOnAlarm(Alarm) error - switches alarm on leaving the alarm time as it was
//
//	TurnOffAlarm(Alarm) error - switches off alarm, but leaves alarm time alone

package ds3231 // import "tinygo.org/x/drivers/ds3231"

import (
	"fmt"
	"time"

	"tinygo.org/x/drivers"
)

// Clocker is an interface to a real-time clock. It is intended to support basic functions
// supported by most clocks.
type Clocker interface {
	Configure() error
	Read() (time.Time, error)
	Set(time.Time) error
	IsTimeValid() bool
	SetAlarm(time.Time, Alarm, AlarmPeriod) error
	SilenceAlarm(Alarm) error
	TurnOnAlarm(Alarm) error
	TurnOffAlarm(Alarm) error
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
	BadTime = iota + 1
	InvalidAlarm
	FailedToReadClock
	FailedWriteToClock
	FailedToConfigureClock
	NotImplemented
)

// Alarm types. Daily matches HH MM SS. Hourly MM SS and Minutely SS.
// Other types may (eventually) be implemented - keep these in order: shortest time first
const (
	Secondly = iota + 1
	Minutely
	Hourly
	Daily
)

// masks
const (
	setAlarmbitOn  = 0b01111111 // and with this
	setAlarmbitOff = 0b10000000 // or with this
	alarm1Enable   = 0b00000001 // or with this
	alarm2Enable   = 0b00000010 // or with this
	alarm1Disable  = 0b11111110 // and with this
	alarm2Disable  = 0b11111101 // and with this
	prepareAlarms1 = 0b00000100 // write to REG_CONTROL
	prepareAlarms2 = 0b00000000 // write to REG_STATUS
	silenceAlarm1  = 0b11111110 // & this with REG_STATUS
	silenceAlarm2  = 0b11111101 // & this with REG_STATUS
	inctnOn        = 0b00000100 // or with this
	inctnOff       = 0b11111011 // & with this
)

func (se StatusErr) Error() string {
	return se.Message
}

type Mode uint8

// Alarm is the alarm number
type Alarm uint8
type AlarmBits byte
type AlarmPeriod uint8

// Device implements the Clocker interface and wraps an I2C connection to a DS3231 device.
type Device struct {
	bus     drivers.I2C
	Address uint16
}

// Read reads the DS3231 and returns a Time
func (d *Device) Read() (time.Time, error) {
	data := make([]uint8, 7)

	dt := time.Time{}
	err := d.bus.ReadRegister(uint8(d.Address), REG_TIMEDATE, data)
	if err != nil {
		return dt, err
	}

	second := bcdToInt(data[0] & 0x7F)
	minute := bcdToInt(data[1])
	hour := hoursBCDToInt(data[2])
	day := bcdToInt(data[4])
	monthRaw := data[5]
	year := bcdToInt(data[6]) + 2000
	if monthRaw&(1<<7) != 0x00 {
		year += 100
	}
	month := time.Month(bcdToInt(monthRaw & 0x7F))

	dt = time.Date(year, month, day, hour, minute, second, 0, time.UTC)
	return dt, nil
}

// Set sets the DS3231 to a UTC time
func (d *Device) Set(dt time.Time) error {
	data := []byte{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_STATUS, data)
	if err != nil {
		return err
	}
	data[0] &^= 1 << OSF
	err = d.bus.WriteRegister(uint8(d.Address), REG_STATUS, data)
	if err != nil {
		return err
	}

	data = make([]uint8, 7)
	data[0] = uint8ToBCD(uint8(dt.Second()))
	data[1] = uint8ToBCD(uint8(dt.Minute()))
	data[2] = uint8ToBCD(uint8(dt.Hour()))

	year := uint8(dt.Year() - 2000)
	centuryFlag := uint8(0)
	if year >= 100 {
		year -= 100
		centuryFlag = 1 << 7
	}

	data[3] = uint8ToBCD(uint8(dt.Weekday()))
	data[4] = uint8ToBCD(uint8(dt.Day()))
	data[5] = uint8ToBCD(uint8(dt.Month()) | centuryFlag)
	data[6] = uint8ToBCD(year)

	err = d.bus.WriteRegister(uint8(d.Address), REG_TIMEDATE, data)
	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write control byte: %s", err)}

		return err
	}

	return nil
}

// SetAlarm sets one of the DS3231 alarms
func (d *Device) SetAlarm(t time.Time, a Alarm, p AlarmPeriod) error {
	if a > 2 || a < 1 {
		return StatusErr{Status: InvalidAlarm,
			Message: fmt.Sprintf("Clock only supports alarm 1 or alarm 2")}

	}

	if a == 2 && p < Minutely {
		return StatusErr{Status: InvalidAlarm,
			Message: fmt.Sprintf("Clock only supports alarm2 periods of minutely/hourly/daily")}
	}

	// fmt.Printf("Alarm time: %02d:%02d:%02d\r\n", t.Hour(), t.Minute(), t.Second())

	// Set up the four registers with the time

	if a == 1 {

		data := make([]uint8, 4)

		data[0] = uint8ToBCD(uint8(t.Second())) | setAlarmbitOff // No alarm matching to start
		data[1] = uint8ToBCD(uint8(t.Minute())) | setAlarmbitOff
		data[2] = uint8ToBCD(uint8(t.Hour())) | setAlarmbitOff
		data[3] = 0 | setAlarmbitOff // date matching not yest implemented

		// Add in the right masks for the chosen period

		if p >= Minutely {
			data[0] &= setAlarmbitOn
		}

		if p >= Hourly {
			data[1] &= setAlarmbitOn

		}
		if p >= Daily {
			data[2] &= setAlarmbitOn
		}
		/* Diag stuff
		for i := 0; i < len(data); i++ {

			fmt.Printf("data[%d] after masking : %08b \n", i, data[i])

		} */

		err := d.bus.WriteRegister(uint8(d.Address), REG_ALARMONE, data)

		if err != nil {
			return StatusErr{Status: FailedWriteToClock,
				Message: fmt.Sprintf("unable to write alarm %d to clock: %s", a, err)}
		}

	} else {
		// alarm 2 logic
		data := make([]uint8, 3)

		data[0] = uint8ToBCD(uint8(t.Minute())) | setAlarmbitOff
		data[1] = uint8ToBCD(uint8(t.Hour())) | setAlarmbitOff
		data[2] = 0 | setAlarmbitOff // date matching not yet implemented

		// Add in the right masks for the chosen period

		if p >= Hourly {
			data[0] &= setAlarmbitOn

		}
		if p >= Daily {
			data[1] &= setAlarmbitOn
		}
		/* // Diag stuff
		for i := 0; i < len(data); i++ {

			fmt.Printf("data[%d] after masking : %08b \n", i, data[i])

		} */
		err := d.bus.WriteRegister(uint8(d.Address), REG_ALARMTWO, data)

		if err != nil {
			return StatusErr{Status: FailedWriteToClock,
				Message: fmt.Sprintf("unable to write alarm %d to clock: %s", a, err)}
		}
	}

	// now switch on the alarm
	err := d.TurnOnAlarm(a)
	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to switch on alarm %d", a, err)}
	}

	/* Diag stuff!
	control := []byte{0}
	for {
		dt, _ := d.Read()

		fmt.Printf("Time: %02d:%02d:%02d\r\n", dt.Hour(), dt.Minute(), dt.Second())
		d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, control)
		fmt.Printf("control byte: %08b\r\n", control)

		d.bus.ReadRegister(uint8(d.Address), REG_STATUS, control)
		fmt.Printf("status byte: %08b\r\n", control)
		time.Sleep(1 * time.Second)
	}
	*/

	return nil
}

// SilenceAlarm clears a sounding alarm so it can be triggered again
func (d *Device) SilenceAlarm(alarm Alarm) error {
	if alarm > 2 || alarm < 1 {
		return StatusErr{Status: InvalidAlarm,
			Message: fmt.Sprintf("Clock only supports alarm = 1 or alarm = 2")}
	}

	status := []byte{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_STATUS, status)
	if err != nil {
		return StatusErr{Status: FailedToReadClock,
			Message: fmt.Sprintf("unable to read status byte: %s", err)}
	}

	if alarm == 1 {
		status[0] = status[0] & silenceAlarm1
	} else { // assume alarm 2
		status[0] = status[0] & silenceAlarm2
	}

	err = d.bus.WriteRegister(uint8(d.Address), REG_STATUS, status)
	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write status byte: %s", err)}
	}

	return nil
}

// Configure sets up the device for communication, makes sure all the alarms are
// off, the interrupt is set and the clock is running
func (d *Device) Configure() error {
	err := d.prepareForAlarms()
	if err != nil {
		return StatusErr{Status: FailedToConfigureClock,
			Message: fmt.Sprintf("unable to configure: %s", err)}
	}
	if !d.isRunning() {
		return StatusErr{Status: FailedToConfigureClock,
			Message: fmt.Sprintf("clock not running")}
	}
	return nil
}

func (d *Device) prepareForAlarms() error {
	// clock always on, alarms active, squarwave off
	control := []byte{prepareAlarms1} // Enable INCTN, and interrupts on both alarms, clear Status

	err := d.bus.WriteRegister(uint8(d.Address), REG_CONTROL, control)
	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write control byte: %s", err)}

	}

	status := []byte{prepareAlarms2}
	err = d.bus.WriteRegister(uint8(d.Address), REG_STATUS, status)
	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write status byte: %s", err)}
	}

	return nil
}

// TurnOnAlarm is for switching an alarm on. It assumes the
// various bitmasks of the alarm bytes have been set.
func (d *Device) TurnOnAlarm(alarm Alarm) error {
	// just to be certain
	err := d.SilenceAlarm(alarm)
	if err != nil {
		return err
	}
	// next rewrite the Control bit to switch the alarm on
	control := []byte{0}
	err = d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, control)
	if err != nil {
		return StatusErr{Status: FailedToReadClock,
			Message: fmt.Sprintf("unable to read control byte: %s", err)}
	}

	if alarm == 1 {
		control[0] = control[0] | alarm1Enable
	} else {
		control[0] = control[0] | alarm2Enable
	}

	err = d.bus.WriteRegister(uint8(d.Address), REG_CONTROL, control)

	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write status byte: %s", err)}
	}

	return nil
}

// TurnOffAlarm is for switching an alarm on. It assumes the
// various bitmasks of the alarm bytes have been set.
func (d *Device) TurnOffAlarm(alarm Alarm) error {
	// just to be certain
	err := d.SilenceAlarm(alarm)
	if err != nil {
		return err
	}
	// next rewrite the Control bit to switch the alarm off
	control := []byte{0}
	err = d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, control)
	if err != nil {
		return StatusErr{Status: FailedToReadClock,
			Message: fmt.Sprintf("unable to read control byte: %s", err)}
	}

	if alarm == 1 {
		control[0] &= alarm1Disable
	} else {
		control[0] &= alarm2Disable
	}

	err = d.bus.WriteRegister(uint8(d.Address), REG_CONTROL, control)

	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write status byte: %s", err)}
	}

	return nil
}

// SetAlarmInterrupt switches the interrupt on (status = True) or off (status = False).
// This is set on by default in Configure
func (d *Device) SetAlarmInterrupt(status bool) error {

	// Read the control byte
	control := []byte{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, control)
	if err != nil {
		return StatusErr{Status: FailedToReadClock,
			Message: fmt.Sprintf("unable to read control byte: %s", err)}
	}

	if status {
		control[0] |= inctnOn
	} else {
		control[0] &= inctnOff
	}

	err = d.bus.WriteRegister(uint8(d.Address), REG_CONTROL, control)

	if err != nil {
		return StatusErr{Status: FailedWriteToClock,
			Message: fmt.Sprintf("unable to write control byte: %s", err)}
	}

	return nil
}

// New creates a new DS3231 connection. The I2C bus must already be
// configured.
//
// This function only creates the Device object, it does not touch the device.
func New(bus drivers.I2C) Device {
	return Device{
		bus:     bus,
		Address: Address,
	}
}

// IsTimeValid return true/false is the time in the device is valid
func (d *Device) IsTimeValid() bool {
	data := []byte{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_STATUS, data)
	if err != nil {
		return false
	}
	return (data[0] & (1 << OSF)) == 0x00
}

// isRunning returns if the oscillator is running
func (d *Device) isRunning() bool {
	data := []uint8{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, data)
	if err != nil {
		return false
	}
	return (data[0] & (1 << EOSC)) == 0x00
}

// setRunning starts the internal oscillator
func (d *Device) setRunning(isRunning bool) error {
	data := []uint8{0}
	err := d.bus.ReadRegister(uint8(d.Address), REG_CONTROL, data)
	if err != nil {
		return err
	}
	if isRunning {
		data[0] &^= uint8(1 << EOSC)
	} else {
		data[0] |= 1 << EOSC
	}
	err = d.bus.WriteRegister(uint8(d.Address), REG_CONTROL, data)
	if err != nil {
		return err
	}
	return nil
}

// ReadTemperature returns the temperature in millicelsius (mC)
func (d *Device) readTemperature() (int32, error) {
	data := make([]uint8, 2)
	err := d.bus.ReadRegister(uint8(d.Address), REG_TEMP, data)
	if err != nil {
		return 0, err
	}
	return int32(data[0])*1000 + int32((data[1]>>6)*25)*10, nil
}

// uint8ToBCD converts a byte to BCD for the DS3231
func uint8ToBCD(value uint8) uint8 {
	return value + 6*(value/10)
}

// bcdToInt converts BCD from the DS3231 to int
func bcdToInt(value uint8) int {
	return int(value - 6*(value>>4))
}

// hoursBCDToInt converts the BCD hours to int
func hoursBCDToInt(value uint8) (hour int) {
	if value&0x40 != 0x00 {
		hour = bcdToInt(value & 0x1F)
		if (value & 0x20) != 0x00 {
			hour += 12
		}
	} else {
		hour = bcdToInt(value)
	}
	return
}

// set nth bit to 1
func setNthBit(data uint8, pos uint8) uint8 {
	return (data | (1 << pos))
}
