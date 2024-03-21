# Tasks Description

### CAN_TX_ISR
This task is implemented by interrupt.
This task is triggered whenever a CAN message is successfully transimitted and after that it release the semaphore allowing other taskes to proceed transmission.

### CANT_RX_ISR
This task is implemented by interrupt.
This task is triggered whenever a CAN message is received and each time it sends the received messaget to msgIn queue.

### CAN_TX_Task
This task is implemented by thread.
Whenever the msgOut queue receives a new message, this task runs to transmit the message to CAN bus.

### DecodeTask
This task is implemented by thread.
Whenever the msgIn queue receives a new message, this task will run to process it. It will send command to other tasks base on the message it receives.

### SampleISR
This task is implemented by interrupt.
This task runs at sample frequency 22kHz, each time it chooses the right value to the speaker and analogueWrite it.

### ScanKeysTask
This task is implemented by thread.
This task runs every 20ms, each time it scans all inputs to the board. And also, this task detects whether it is in a sequence of connected boards. If it was, but now it's not, this task will reset the board to local play mode. If it was not, but now it is, this task will send update request to update its position.

### DisplayUpdateTask
This task is implemented by thread.
This task runs every 100ms, each time it updates the disaplay in the screen. This task is responsible for all display and UI menu, it will adjust values of settings and shows corresponding page based on user inputs.


### ScanJoystickTask
This task is implemented by thread.
This task frequently reads the input from Joystick and outputs its movement base on that.

### Auto-Detection
This task is a normal function that only run once in setup.
This task is run when the board starts, then ouputs the position id of each board by exchanging information through West/East detection signal and CAN bus.


### BackgroundCalcTask
This task is implemented by thread.
