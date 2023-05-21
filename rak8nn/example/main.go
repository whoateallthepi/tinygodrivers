package main

import (
	"fmt"
	"time"

	"github.com/whoateallthepi/tinygodrivers/rak8nn"
)

// Interface for lorawan
type lorawan rak8nn.Networker

/*
func join(n rak8nn.Networker) error {
	err := n.Join()
	return err
}

func status(n rak8nn.Networker) (uint8, error) {
	return 0, nil
}

func upload(n rak8nn.Networker, data []byte, channel uint8) (*rak8nn.DataBlock, error) {

	db, err := n.Send(data, channel)

	if err != nil {
		return nil, rak8nn.StatusErr{
			Status:  rak8nn.FailedtoSend,
			Message: fmt.Sprintf("failed to send: %s", err),
		}
	}
	return db, nil
}
*/

/* This is a simple example of using the library.
* 1) Create a devce
* 2) Join the loraWan network
* 3) Upload, and optionally download some data
*
* This all assumes you have a corectly configured RAK811 module, correctly connected to the
* right UART. For instructions on that - see eleswhere.
* I have mainly used this with GDB to inspect variables etc. You may wih to improve the
* USB port output (from fmt.Print etc if you wish to develop further.)
 */

type networker interface {
	Initialise() error
	Join() error
	Send(sendData []byte, sendChannel uint8) (channel uint8, // channel
		rssi int16, // rssi
		snr int16, // Snr
		bytes uint8, // byte count
		data []byte, // data
		err error)
}

type network struct {
	network networker
}

func newNetwork(networker networker) *network {
	return &network{
		network: networker,
	}
}

func main() {

	// Give me time too get terminal started
	for i := 0; i < 10; i++ {
		fmt.Printf("rak8nn example\n")
		time.Sleep(time.Second)
	}

	//time.Sleep(10 * time.Second)

	// create the device
	d, err := rak8nn.NewDevice("RAK811", 0, 115200)

	if err != nil {
		fmt.Println(err)
		return
	}
	// create the network - device should implement interface
	n := newNetwork(d).network

	// Connect device to network
	fmt.Printf("Joining network\n")

	err = n.Join()

	if err != nil {
		fmt.Printf("Failed to connect to network \n", err)
		for {
		}
	}

	fmt.Printf("Joined network\n")

	// If you are testing downlinks - stop with debugger here and
	// send some downlink data (eg via TTN)

	// Send some test data
	var data []byte = []byte("aabbcc")
	var cha uint8 = 100

	fmt.Printf("Trying uplink \n")
	c, r, s, b, up, err := n.Send(data, cha)

	fmt.Printf("after uplink\n")

	if err != nil {
		fmt.Printf("uplink failed: %s\n", err)
		for { // not carrying on but this ensures we get messages before end
		}
	}

	// Print the return data block
	fmt.Printf("Downlink data:\n channel: %d\n, rssi: %d\n, snr: %d\n, bytes: %d\n, data: %s\n",
		c, r, s, b, up)

	for {
	}

}
