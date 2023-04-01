// Connects to an ds3231 real time clock.
package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/ds3231"
)

// masks
const ()

func main() {
	alarmChannel := make(chan int, 1)
	alarm := machine.GP3
	alarm.Configure(machine.PinConfig{Mode: machine.PinInput})

	machine.I2C0.Configure(machine.I2CConfig{SCL: machine.GP21,
		SDA: machine.GP20,
	})

	rtc := ds3231.New(machine.I2C0)
	rtc.Configure()

	go func() {
		for {
			_ = <-alarmChannel
			dt, err := rtc.Read()
			if err != nil {
				fmt.Println("Error reading date:", err)
			} else {
				fmt.Printf("Date: %d/%s/%02d %02d:%02d:%02d \r\n", dt.Year(), dt.Month(), dt.Day(), dt.Hour(), dt.Minute(), dt.Second())
			}

			time.Sleep(time.Second)

			rtc.SilenceAlarm(1)

		}

	}()

	valid := rtc.IsTimeValid()
	valid = false
	if !valid {
		date := time.Date(2022, 12, 05, 20, 34, 12, 0, time.UTC)
		rtc.Set(date)
	}

	running := rtc.IsRunning()
	if !running {
		err := rtc.SetRunning(true)
		if err != nil {
			fmt.Println("Error configuring RTC")
		}
	}

	for x := 0; x < 5; x++ {
		dt, err := rtc.Read()
		if err != nil {
			fmt.Println("Error reading date:", err)
		} else {
			fmt.Printf("Date: %d/%s/%02d %02d:%02d:%02d \r\n", dt.Year(), dt.Month(), dt.Day(), dt.Hour(), dt.Minute(), dt.Second())
		}
		temp, _ := rtc.ReadTemperature()
		fmt.Printf("Temperature: %.2f C \r\n\n", float32(temp)/1000)

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

	fmt.Println("trying to set alarm - every second ")
	dt, err := rtc.Read()
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	err = rtc.SetAlarm(dt.Add(time.Second*10), 1, ds3231.Secondly) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		return
	}
	time.Sleep(time.Second * 20)
	fmt.Println("Switching alarm off")
	time.Sleep(time.Second * 1)

	rtc.TurnOffAlarm(1)

	fmt.Println("trying to set alarm - every minute")

	dt, err = rtc.Read()
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	err = rtc.SetAlarm(dt.Add(time.Second*10), 1, ds3231.Minutely) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		return
	}

	time.Sleep(time.Minute * 5)

	fmt.Println("trying to set alarm - every hour ")
	dt, err = rtc.Read()
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	err = rtc.SetAlarm(dt.Add(time.Second*10), 1, ds3231.Hourly) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		return
	}
	time.Sleep(time.Minute * 190)
	fmt.Println("Switching alarm off")
	time.Sleep(time.Second * 1)
	rtc.TurnOffAlarm(1)

	fmt.Println("trying to set alarm - every Day")

	dt, err = rtc.Read()
	if err != nil {
		fmt.Println("Error reading date:", err)
		return
	}

	err = rtc.SetAlarm(dt.Add(time.Second*10), 1, ds3231.Daily) // Set time for 10s in future

	if err != nil {

		fmt.Println("Error setting alarm:", err)
		return
	}

	time.Sleep(time.Hour * 250) // Just wait for interrupts...

}
