package main

import (
	"machine"
	"time"

	"github.com/whoateallthepi/tinygodrivers/bme280spi"
)

func main() {
	time.Sleep(5 * time.Second)

	machine.SPI0.Configure(machine.SPIConfig{})
	sensor := bme280spi.NewSPI(machine.A5, machine.SPI0)
	sensor.Configure()

	/*
		if !sensor.Connected() {
			println("BME280 not connected")
			return
		}
	*/
}
