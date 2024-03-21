# SpaceY Dependencies

The table below 
Empty means not using that data
| Data Name          | sysState | settings    | movement    | notes | 
|--------------------|----------|-------------|-------------|-------|
| ScanKeysTask       | Mutex    | Mutex       | Atomic Load | Mutex |
| DisplayUpdateTask  | Mutex    | Mutex       | Atomic Load | Null  |
| BackgroundCalcTask | Null     | Atomic Load | Null        | Atomic Load |
| ScanJoystickTask   | Null     | Null        | Atomic Store| Null  |
| DecodeTask         | Mutex    | Mutex       | Null        | Mutex |
| Sample ISR         | Atomic Load|Atomic Load|Null|Null|

All RX&TX Messages are protected by message queue and semaphore.
Two different buffers which are shared by BackCal and Sample ISR, buffers are protected by semephore.

Intertask blocking dependencies:

In the above graph, Display task, sampleISR, backcalc task is dependednt on systate, settings and notes. When the structs are accessed, atomic loads or mutex are all used to prevent deadlocks and improve data security. 

Some examples of data flow and dependency:

A knob is changed to change tone: ScanKeyTask (change systate)-> CANTX->CANRX->Decode task (changes settings)->backcal to use settings to generate sound

A joystick is moved to change menu: joysickscantask(changes movement)->displayupdate(changes update accurdingly)

No deadlocks because the functions that reads to those structs do not have ability to write back to those blocks. Even if they do, the part of the data they write back to are only accessed by themselves and not other structs. Some mistake can be caused due to timing problems of a task not being able to finish, therefore not being able to update the value for the following functions to read, but that does not cause deadlocks.