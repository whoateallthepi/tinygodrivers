package main

import (
	"fmt"
	"time"

	"github.com/whoateallthepi/tinygodrivers/rak8nn"
)

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

func main() {

	d, err := rak8nn.NewDevice("RAK811", 0, 115200)

	// Give me time too get terminal started
	time.Sleep(10 * time.Second)

	if err != nil {
		fmt.Println(err)
		return
	}

	err = join(d)

	if err != nil {
		fmt.Println(err)
		return
	}

	var data []byte = []byte("aabbcc")
	var cha uint8 = 100

	r, err := upload(d, data, cha)

	if err != nil {
		fmt.Println(err)
		return
	}
	fmt.Println(r) // The return data.
}
