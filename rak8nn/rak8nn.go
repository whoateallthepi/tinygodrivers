package rak8nn

import (
	"bytes"
	"errors"
	"fmt"
	"machine"
	"strconv"
	"strings"
	"time"
)

// Networker is the interface for networks - typically a LoraWan network
type Networker interface {
	//Configure()
	Initialise() error
	Join() error
	Send([]byte, uint8) (*DataBlock, error)
	Status() (uint8, error)
	//Receiveno required module provides package()
}

type DataBlock struct {
	channel uint8
	rssi    int16
	snr     int16
	bytes   uint8
	data    []byte
}

// Status is used for error reporting - see StatusErr
type Status int

// Error codes
const (
	ModemError = iota + 1
	FailedtoConnect
	FailedtoSend
	UnexpectedModemResponse
	UnrecoverableError
)

const (
	joinRetries = 5 // Number of times to try joiniing LoraWan network
)

// StatusErr is an implementation of error interface to include a Status integer
type StatusErr struct {
	Status  Status
	Message string
}

func (se StatusErr) Error() string {
	return se.Message
}

const (
	joinCommand    = "at+join\r\n"
	versionCommand = "at+version\r\n"
	uploadCommand  = "at+send=lora:%d:%s\r\n"
	dataMessage    = "at+recv="
	generalOK      = "OK"
	errorResponse  = "ERROR:"
)

// Device is an implementation of Interface Network for RAK8nn devices - currently only RAK811 supported
type Device struct {
	uart *machine.UART
}

/*
func ( device *Device) Configure (baudRate uint32) error {
	config := machine.UARTConfig {
		BaudRate: baudRate,

	}

}
*/

// Join an RAK8nn device to a LoraWan network
func (d *Device) Join() error {

	var (
		connected bool = false
	)

	//try connecting up to five times

	for i := 0; i < joinRetries; i++ {
		cr, data, mc, err := d.command([]byte(joinCommand), 10)
		fmt.Print(cr, data, mc, err)

		if err == nil {
			connected = true
			break
		}

	}

	fmt.Print(connected)

	if connected {
		return nil
	}

	return StatusErr{
		Status:  FailedtoConnect,
		Message: "failed to connect to network",
	}
}

// Send (or 'upload') data
func (d *Device) Send(data []byte, channel uint8) (*DataBlock, error) {

	var command []byte
	command = []byte(fmt.Sprintf(uploadCommand, channel, data))

	fmt.Print(command)
	_, dd, mc, err := d.command([]byte(command), 10) // discard command response (debugging only)

	if err != nil {
		return nil, StatusErr{
			Status:  FailedtoSend,
			Message: fmt.Sprintf("failed to connect to network modem code: %d", mc),
		}
	}

	return dd, nil
}

// Status is the status of a device to the network
func (*Device) Status() (uint8, error)

/* Currently we only support Class A lorawan devices - so there is no 'receive' mode
 * data is download immediately after and upload
func (*Device) Receive() {
else {
}
*/

// Initialise does whatever i needed to get device ready for comms. At very least clears out
// any unread bytes from I/O buffer
func (d *Device) Initialise() error {

	// Send a 'version' command - this wakes up the modem and we can read out the buffers

	commandResponse, data, modemCode, err := d.command([]byte(versionCommand), 0)
	fmt.Println(commandResponse)
	fmt.Println(modemCode)
	fmt.Println(err)
	fmt.Println(data)

	return err
}

// command is a utility to send an at-type command to the modem and interpret the response
// also passes back the text of the response (commandResponse) to help with debugging/tracing
// As this is a Cat A device, some data may be returned after an uplink - this is returned in data.
// All commands are a bit slow - join in particular takes seconds.
func (d *Device) command(command []byte, responseDelay uint32) (commandResponse []byte, data *DataBlock, modemCode int64, err error) {

	// No validation at this stage!

	//response := make([]byte, 10)

	d.uart.Write(command)

	time.Sleep(time.Second) // Brief pause....

	// we should always get something back from the modem - allow up to 60s
	// for this

	for x := 0; x < 60; x++ {
		if d.uart.Buffered() > 0 {
			break
		}
		time.Sleep(time.Second)
	}

	//time.Sleep(time.Second) // Brief pause....

	// read out the buffer *should* end in \r\n
	n := d.uart.Buffered()

	for n > 0 {
		inByte, _ := d.uart.ReadByte()
		// Ignore the 'break' character from buffer
		if inByte != 0xf0 {
			commandResponse = append(commandResponse, inByte)
		}
		time.Sleep(50 * time.Millisecond) // Slow it down a bit
		n = d.uart.Buffered()
	}
	// Check for ERROR: nn type responses

	fmt.Print(commandResponse)

	if len(commandResponse) > 8 {

		if bytes.Equal([]byte(errorResponse), commandResponse[:len(errorResponse)]) {
			// Try and get error code (one or two digits) from the modem

			i, err := strconv.Atoi(string(commandResponse[7]))
			if err == nil {
				i2, err := strconv.Atoi(string(commandResponse[8]))
				if err == nil {
					modemCode = (10 * int64(i)) + int64(i2)
				} else {
					modemCode = int64(i)
				}
			}

			return commandResponse, nil, modemCode, StatusErr{
				Status:  ModemError,
				Message: fmt.Sprintf("error code from modem: %d", modemCode),
			}

		}
	}

	// check for an OK response

	fmt.Print(commandResponse)

	if len(commandResponse) < 2 {
		// no idea what this is
		return commandResponse, nil, 0, StatusErr{
			Status:  UnexpectedModemResponse,
			Message: fmt.Sprintf("unrecognised response from modem: %s", commandResponse),
		}
	}

	if bytes.Equal([]byte(generalOK), commandResponse[:len(generalOK)]) {
		// Read on to check if we also have a data message

		start := strings.Index(string(commandResponse), dataMessage)
		if start >= 0 {
			// We have data - parse it. This assumes we do not have any UTF-8 values
			// with multi-byte characters
			dd := make([]byte, 200)

			splits := strings.Split(string(commandResponse[start+len(dataMessage):]), ",")

			i0, _ := strconv.ParseInt(string(splits[0]), 10, 16)
			i1, _ := strconv.ParseInt(string(splits[1]), 10, 16)
			i2, _ := strconv.ParseInt(string(splits[2]), 10, 16)
			x3 := strings.TrimSpace((splits[3])) // get rid of \r\n

			i3, err := strconv.ParseInt(x3, 10, 16)
			if err != nil {
				//x3 probably  has some data eg we received at+recv=0,-82,5,2:aabb so first get the
				// number before the colon
				s2 := strings.Split(x3, ":")
				i3, _ = strconv.ParseInt(s2[0], 10, 16) // Bit before :
				dd = []byte(s2[1])                      // What's left should be the data in hex
			}

			db := DataBlock{channel: uint8(i0),

				rssi:  int16(i1),
				snr:   int16(i2),
				bytes: uint8(i3),
				data:  dd,
			}
			return commandResponse, &db, 0, nil
		}
		// This is the return for no data from network, eg a at+version\r\n command

		return commandResponse, nil, 0, nil
	}

	// We rshould have processed all  the possible responses from modem
	// - so return an error if we get here

	return commandResponse, data, modemCode, StatusErr{
		Status:  UnexpectedModemResponse,
		Message: fmt.Sprintf("unrecognised response from modem: %s", commandResponse),
	}

}

// NewRak8nnDevice is a generator fuction to create an RAK8nn device instance
func NewDevice(deviceType string, uart uint8, baudRate uint32) (*Device, error) {
	if deviceType != "RAK811" {
		return nil, errors.New("Currently RAK811 is only supported device")
	}

	var selectedUART *machine.UART
	var tx machine.Pin
	var rx machine.Pin

	if uart == 0 {
		selectedUART = machine.UART0
		tx = machine.UART0_TX_PIN
		rx = machine.UART0_RX_PIN
	} else if uart == 1 {
		selectedUART = machine.UART1
		tx = machine.UART1_TX_PIN
		rx = machine.UART1_RX_PIN
	} else {
		return nil, errors.New("Only Uarts 0 and 1 available")

	}

	selectedUART.Configure(machine.UARTConfig{
		BaudRate: baudRate,
		TX:       tx,
		RX:       rx,
	})

	d := Device{
		uart: selectedUART,
	}

	err := d.Initialise()

	return &d, err

}

func join(n Networker) error {
	err := n.Join()
	return err
}

func status(n Networker) (uint8, error) {
	return 0, nil
}

func upload(n Networker, data []byte, channel uint8) (*DataBlock, error) {

	db, err := n.Send(data, channel)

	if err != nil {
		return nil, StatusErr{
			Status:  FailedtoSend,
			Message: fmt.Sprintf("failed to send: %s", err),
		}
	}
	return db, nil
}
