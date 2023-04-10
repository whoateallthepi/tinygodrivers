// ledpanel provides control of a panel of up to 4 leds. A goroutine is created and a channel returned.
// Sending a control byte (type LedControl) to the channel triggers various led flashes. The
// goroutine will terminate when the channel is closed.
//
// Why use this?
// *************
// With microcontroller projects I often use leds to report errors and/or status. Unfortunately, by the end of the
// project I often end up with different error codes coded in dufferent parts of the project in different ways,
// with little consistency.
//
// Using ledpanel, a set of error codes can be decided at the start (or developed during the project)
// then changing an error code is just a matter of amending a constant.
//
// For example
// ***********
// If we decide three very long flashes of three leds signals bootup, then it can just be
// const ( bootOK = 0b10110111). Stuffing that byte to the led channel (returned by ConfigureLedPanel)
// will flash that code. If, at some stage, we decide flashing just led0 and led3 would be better
// then we can amend bootOK to 0b10111001. It is as 'simple' as that.
//
// Masks
// *****
// Some basic masks are in the API to make this simple eg led0 | led3 | verylong | threeFlash... etc
//
// The pins are selected by a slice of up to four machine.Pins supplied to ConfigureLedPanel.
// If you have only 1, 2 or 3 leds, just  supply those pins (via LedPanel). Any requests to
// light up unconfigured leds will be ignored.

package ledpanel

import (
	"errors"
	"fmt"
	"machine"
	"time"
)

// Bitmasks

const (
	// Led0 is a bitpattern to light led0
	Led0 = 1 << iota
	// Led1 is a bitpattern to light led1
	Led1
	// Led2 is a bitpattern to light led2
	Led2
	// Led3 is a bitpattern to light led3
	Led3
	// Short is a bitpattern for a short flash
	Short = 0b00000000
	// Medium is a bitpattern for a medium flash
	Medium = 0b00010000
	// Long is a bitpattern for a long flash
	Long = 0b00100000
	// VeryLong is a bitpattern for a very long flash
	VeryLong = 0b00110000
	// OneFlash is a bitpattern to flash the leds once
	OneFlash = 0b00000000
	// TwoFlash is a bitpattern to flash the leds twice
	TwoFlash = 0b01000000
	// ThreeFlash is a bitpattern to flash the leds three times
	ThreeFlash = 0b10000000
	// FourFlash is a bitpattern to flash the leds four times
	FourFlash = 0b11000000

	// masks
	durationBits    = 0b00110000
	numberofFlashes = 0b11000000
	selectedLeds    = 0b00001111
	// limits
	pinLimit      = 4 // There are only 4 bits for pin control (0 to 3)
	durationLimit = 4
)

const (
	shortflash    = 5 // milliseconds
	mediumflash   = 10
	longflash     = 20
	verylongflash = 50
	flashinterval = 250 // gap between flashes
)

// LedControl is a single byte used to control which leds flash,
// how many times and for how long.
//
// Bits:
//
//	7   6 		| 5   4			| 3   2    1    0
//
//	Flashes		Duration		 LED3 LED2 LED1 LED0
//	0 = 1		0 = short		 0 = off 1 = on
//	1 = 2		1 = medium
//	2 = 3		2 = long
//	3 = 4		3 = very long
type LedControl uint8

// Leds is used to hold the pins to which the leds are connected -
// used in LedPanel struct
type Leds []machine.Pin

// FlashDurations contains the durations for each plash length - see LedPanel
type FlashDurations [durationLimit]uint8

// LedPanel defines a panel of up to 4 Leds. LedPins is the hardware pins to which
// the leds are connected. Durations is the time intervals
// for which the leds are lit in milliseconds.
//
// Durations 		(defaults)
//
//	0 = short			5
//	1 = medium			10
//	2 = long			20
//	3 = very long		50
//
// If any of the durations are omitted, defaults will be used

// LedPanel is used to initialise the leds.
//
//	ledPins : a slice of up to 4 machine.Pins with leds connected
//	durations : an array of four integers corresponding to
//				the legnth of flashes for the leds in milliseconds
//				durations[0] = shortFlash
//				durations[1] = mediumFlash
//				durations[2] = longflash
//				durations[3] = veryLongFlash
type LedPanel struct {
	ledPins   Leds
	durations FlashDurations
}

// ConfigureLedPanel starts a goroutine and returns a channel to which a control byte
// (type LedControl) is sent // to flash an led panel. Uses a goroutine that terminates when the channel is closed
func ConfigureLedPanel(p LedPanel) (chan<- LedControl, error) {

	if len(p.ledPins) > pinLimit {
		return nil, errors.New("more than four pins supplied")
	}

	for _, l := range p.ledPins {
		l.Configure(machine.PinConfig{Mode: machine.PinOutput})
		l.Low()
	}

	// add any un-allocated durations

	for k, v := range p.durations {
		if v != 0 {
			continue
		}

		switch k {
		case 0:
			p.durations[0] = shortflash
		case 1:
			p.durations[1] = mediumflash
		case 2:
			p.durations[2] = longflash
		case 3:
			p.durations[3] = verylongflash
		}

	}

	cc := make(chan LedControl)

	// fire off the go routine to 'listen' for control bytes
	// need to add tidy up code

	go flashLeds(p, cc)

	return cc, nil

}

// flashLeds reads from the channel and flashes the requested leds
func flashLeds(p LedPanel, control <-chan LedControl) error {
	fmt.Println(">> flashLeds")
	for {
		cc, ok := <-control
		if !ok {
			//channel closed
			return nil
		}

		c := uint8(cc)

		fmt.Printf("c: %d\n", c)

		// get flashes

		var flashes, ledBits, i, mask uint8
		var duration time.Duration

		flashes = 1 + (c&numberofFlashes)>>6
		d := c & durationBits >> 4

		duration = time.Duration(p.durations[d]) * time.Millisecond

		ledBits = c & selectedLeds

		fmt.Printf("flashes:%d duration: %s ledBits: %d\n", flashes, duration, ledBits)

		// Time to flash...
		for i = 0; i < flashes; i++ {
			// light up the LEDs where the bit is set
			for i, ll := range p.ledPins {
				mask = (1 << i)
				if ledBits&mask > 0 {
					ll.High()
				}
			}

			time.Sleep(duration)

			// Now switch off
			for i, ll := range p.ledPins {
				mask = (1 << i)
				if ledBits&mask > 0 {
					ll.Low()
				}

			}

			time.Sleep(time.Millisecond * flashinterval)
		}

	} // loops until the channel is closed

}
