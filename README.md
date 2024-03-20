# ES-synth-starter

  Use this project as the starting point for your Embedded Systems labs and coursework.
  
  [Lab Part 1](doc/LabPart1.md)
  
  [Lab Part 2](doc/LabPart2.md)

## Additional Information

  [Handshaking and auto-detection](doc/handshaking.md)
  
  [Double buffering of audio samples](doc/doubleBuffer.md)

  [StackSynth V1.1 Schematic](doc/StackSynth-v1.pdf)

  [StackSynth V2.1 Schematic](doc/StackSynth-v2.pdf)



timing analysis of background caclculation:
15181
30383
45575
60762
75947
91133
106321
121510
136701
151890
167079
182268
197458
212648
227841
243030
258219
273408
288597
303788
318977
334166
349358
364547
379739
394928
410117
425306
440495
455687
470879
486068
number of microseconds of each iteration, 15.2 ms each iteration rounded up in the worst case senerio genreating a full sample that fills the array. The worst case senerio consists of 48 keys being pressed at the same time, and with additional features all added on using defualt settings.
The theoretical initaition interval is 550/22000=25ms, this is the time taken for the sample isr to read 