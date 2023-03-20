package main

import (
	"fmt"
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/bme280spi"
)

// Interface for sensor
type sensor bme280spi.Sensor

//type sensorReading bme280spi.SensorReading

func main() {

	time.Sleep(time.Second * 5) // time to get terminal up

	fmt.Printf("bme280spi driver example \n")
	fmt.Printf("************************ \n")
	machine.SPI0.Configure(machine.SPIConfig{})

	// Create an instance of the sensor device
	d := bme280spi.NewSPI(machine.GP17, machine.SPI0, 1) // usage 1 = weather 3 = indoor

	// Set it up
	err := sensor.Configure(d)
	fmt.Print(err)
	if err != nil {
		fmt.Printf("Error configuring sensor: %s\n", err)
	}

	for {

		sr, err := sensor.Read(d)
		if err != nil {
			fmt.Printf("Error reading sensor: %s\n", err)
			return
		}
		x := sr.Temperature
		y := sr.Pressure
		z := sr.Humidity

		time.Sleep(time.Second * 10) // Wait a long time!

		fmt.Printf("\nTemperature: %4.2f C\n", float64(x)/1000)
		fmt.Printf("Pressure: %4.2f mB\n", float64(y)/100000)
		fmt.Printf("Humidity: %4.2f %%\n", float64(z)/100)

		time.Sleep(time.Second * 5)
	}
}
