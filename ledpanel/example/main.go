// Example of how to use ledpanel tinyGo driver
package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/ledpanel"
)

func main() {
	for i := 0; i < 5; i++ {
		fmt.Println("Hello")
		time.Sleep(time.Second * 2)
	}

	panel := ledpanel.Panel{Pins: ledpanel.Pins{machine.GP13, machine.GP14, machine.GP15},
		Durations: ledpanel.FlashDurations{3, 7},
	}

	cc, _ := ledpanel.Configure(panel)

	// quick test
	var ll ledpanel.Control
	ll = ledpanel.Medium | ledpanel.ThreeFlash | 0b00000111 // Three leds
	time.Sleep(time.Second)
	for {

		cc <- ll
		time.Sleep(5 * time.Second) // write to channel
		break
	}

	// loop test
	var l, d, f, lc ledpanel.Control
	for { // repeat forever
		for l = 7; l > 0; l-- { // 7 for 3 leds 15 for 4 leds
			for d = 0; d < 4; d++ {
				for f = 0; f < 4; f++ {
					lc = l + (d << 4) + (f << 6)
					fmt.Printf("lc: %d\n", lc)
					cc <- lc
					time.Sleep(time.Second * 2)
				}
			}

		}
	}
}
