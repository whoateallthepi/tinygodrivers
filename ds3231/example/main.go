// Connects to an ds3231 real time clock.
package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/ds3231"
)

// masks
const (
	alarmNumber  = 1
	interruptPin = machine.GP3
)

type clock ds3231.Clocker // Interface

func main() {
	alarmChannel := make(chan int, 1)

	// Configure the interrupt pi for the alarm
	alarm := interruptPin
	alarm.Configure(machine.PinConfig{Mode: machine.PinInput})

	machine.I2C0.Configure(machine.I2CConfig{SCL: machine.GP21,
		SDA: machine.GP20,
	})

	d := ds3231.New(machine.I2C0) // device implementing clocker interface
	clock.Configure(&d)

	// Rountine to receive and acknowledge the alarm going off
	go func() {
		for {
			_ = <-alarmChannel
			dt, err := clock.Read(&d)
			if err != nil {
				fmt.Println("Error reading date:", err)
			} else {
				fmt.Printf("Date: %d/%s/%02d %02d:%02d:%02d \r\n", dt.Year(), dt.Month(), dt.Day(), dt.Hour(), dt.Minute(), dt.Second())
			}

			time.Sleep(time.Second)

			clock.SilenceAlarm(&d, alarmNumber)

		}

	}()

	valid := clock.IsTimeValid(&d)
	//valid = false
	if !valid {
		date := time.Date(2022, 12, 05, 20, 34, 12, 0, time.UTC)
		clock.Set(&d, date)
	}
	/*
		running := rtc.IsRunning()
		if !running {
			err := rtc.SetRunning(true)
			if err != nil {
				fmt.Println("Error configuring RTC")
			}
		}
	*/
	fmt.Println("Reading the clock a few times")
	for x := 0; x < 5; x++ {
		dt, err := clock.Read(&d)
		if err != nil {
			fmt.Println("Error reading date:", err)
		} else {
			fmt.Printf("Date: %d/%s/%02d %02d:%02d:%02d \r\n", dt.Year(), dt.Month(), dt.Day(), dt.Hour(), dt.Minute(), dt.Second())
		}

		time.Sleep(time.Second * 5)
	}

	// Set up alarm interrupt - then clear for next time....
	alarm.SetInterrupt(machine.PinFalling, func(p machine.Pin) {
		fmt.Printf("\n ***** alarm fired****** \n")
		alarmChannel <- 1

		/*
			dt, err := rtc.Read()
			if err != nil {
				fmt.Println("Error reading date:", err)
			} else {
				fmt.Printf("Date: %d/%s/%02d %02d:%02d:%02d \r\n", dt.Year(), dt.Month(), dt.Day(), dt.Hour(), dt.Minute(), dt.Second())
			}
		*/
		//rtc.SilenceAlarm(1)
	})

	fmt.Println("\ntrying to set alarm - every second ")
	banner()

	dt, err := clock.Read(&d)
	dt = dt.Add(time.Second * 10)
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}
	fmt.Printf("Alarm time: %02d:%02d:%02d\n", dt.Hour(), dt.Minute(), dt.Second())
	err = clock.SetAlarm(&d, dt, alarmNumber, ds3231.Secondly) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		fmt.Println("Trying next test")
	}
	time.Sleep(time.Second * 10)

	fmt.Println("Switching alarm off")
	clock.TurnOffAlarm(&d, alarmNumber)

	fmt.Println("\ntrying to set alarm - every minute")
	banner()

	dt, err = clock.Read(&d)
	dt = dt.Add(time.Second * 10)
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	fmt.Printf("Alarm time: %02d:%02d:%02d\n", dt.Hour(), dt.Minute(), dt.Second())
	err = clock.SetAlarm(&d, dt, alarmNumber, ds3231.Minutely) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		fmt.Println("Trying next test")
	}
	time.Sleep(time.Second * 200)

	clock.TurnOffAlarm(&d, alarmNumber)
	time.Sleep(time.Minute * 1)

	fmt.Println("\ntrying to set alarm - every hour ")
	banner()
	dt, err = clock.Read(&d)
	dt = dt.Add(time.Second * 10)
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	fmt.Printf("Alarm time: %02d:%02d:%02d\n", dt.Hour(), dt.Minute(), dt.Second())
	err = clock.SetAlarm(&d, dt, alarmNumber, ds3231.Hourly) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		fmt.Println("Trying next test")
	}
	time.Sleep(time.Minute * 190)

	fmt.Println("Switching alarm off")
	time.Sleep(time.Second * 1)
	clock.TurnOffAlarm(&d, alarmNumber)

	fmt.Println("\ntrying to set alarm - every Day")
	banner()

	dt, err = clock.Read(&d)
	dt = dt.Add(time.Second * 10)
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	fmt.Printf("Alarm time: %02d:%02d:%02d\n", dt.Hour(), dt.Minute(), dt.Second())
	err = clock.SetAlarm(&d, dt, alarmNumber, ds3231.Daily) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		fmt.Println("Trying next test")
	}

	time.Sleep(time.Hour * 250) // Just wait for interrupts...

}
func banner() {
	fmt.Println("*******************************************\n")
}
