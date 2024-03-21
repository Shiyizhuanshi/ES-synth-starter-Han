# ES-synth-starter

  Use this project as the starting point for your Embedded Systems labs and coursework.
  
  [Lab Part 1](doc/LabPart1.md)
  
  [Lab Part 2](doc/LabPart2.md)

## Additional Information

  [Handshaking and auto-detection](doc/handshaking.md)
  
  [Double buffering of audio samples](doc/doubleBuffer.md)

  [StackSynth V1.1 Schematic](doc/StackSynth-v1.pdf)

  [StackSynth V2.1 Schematic](doc/StackSynth-v2.pdf)

worst case senrio of back calc: 48 keys all pressed at the same time with effects added for every single one of the keys pressed. With sample buffer size= 2200, theoretical initiation interval is 1100/22000=50 ms to fill all of the buffer, and the maximum excution time 69.8ms, which is larger. By increasing the sample buffer size to 3300, the time sed also increases to 104000 still > 75ms. it is about 140% over timing. The maximum number of keys can press at the same time is 28.

Worst case senrio of displaytask:
Worst case is whe in the main menu, and all the keys have been pressed, it has to display them all. The time taken to do a full refresh is around 17.2ms, it has a set interval of 1.

Worst case senerio for joysticktask:
initaition itnerval of 100ms, and max time excution is 0.33ms.

Wost case senerio for scankeys:
Worst case is when every single button is pressed, which the input is all 0. 
Each iteration takes about 0.112ms, a lot faster than the set 20ms.

worst case for canTX:
The queue size is 48, in the scankeys, at most 12 messages will be generated every 20ms, therefore a message is pushed into the queue every 1.67 seconds, so 80ms in total for the canTX task when it waits for the queue to fill up. We want to measure the exucion time of it transfering 48 messages at one time, and that averages to 46.3 ms.


worst case for canRX and decodetask:
for the main board, at most 48 rx messages it can recieve at once, from all 4 boards ( it sends 12 messages to the right board and gets transfered back), the decode task is also governed by a recieve queue size 48, the can bus fram is 0.7ms, so 33.6ms for decodetask. By letting the decode task decode 48 times, resulting in 0.014ms.

therefore the worst case tasks and time used are:

decode task: Initiation time: 33.6ms excution time 0.014ms;
canTX task: Initiation of 100ms, excution 58.0ms
joystick task: Initiation time 100ms, excution 0.33ms
scan keys: Initation time 20ms, excution 0.112ms
Display: Initiation 100ms, excution 17.2ms
backcalc all features on: when only 1 board by itself: Initiation 50ms, excution 47.2ms
                          4 boards: Initiation 50ms, excution 108.8ms. 
backcalc all features off: 
                          1 board: Initation 50, excution  33.9ms
                          2 boards: Initiation 50ms, excution 46.2ms
                          4 boards: Initiation 50ms, excution 70.3ms. 

rank them by piorities:
scankey 6 decode 5 backcalc 4 cantx 3 display 2 joystick 1

Critical Instant Analysis
lowest initiation is joystick and display, 100ms,
in 100ms, ~3xDecode task, 1x canTX task, 1x joystick, 5x scankey, 2x backcalc ( using 1 board backcalc)
=3x0.014+1*58.0+5x0.112+1*17.2+2*47.2=160.202 > 100

with all the features turned off
=3x0.014+1*58.0+5x0.112+1*17.2+2*33.9=135.802 > 100
the problem lies within the worst time 

CPU Utilization: 0.014/33.6+58/100+0.33/100+0.112/20+17.2/100+47.2/50=