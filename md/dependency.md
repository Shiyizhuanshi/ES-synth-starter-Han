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