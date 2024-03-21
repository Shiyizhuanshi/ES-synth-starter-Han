|Task no|Task|Execution time $T_{i}$ (ms)|Initiation time $\tau_{i}$ (ms)|$\frac{\tau_{7}T_{i}}{\tau_{i}}$|$\frac{T_{i}}{\tau_{i}}$|
| -------- | :--------: | :-------: | :--------: | :--------: | :--------: |
|1|Decode|0.014|33.6|0.041666667|0.04%|
|~~2~~|~~CAN_TX with propagation delay~~|~~58.0~~|~~100~~|~~58~~|~~58.00%~~|
|3|CAN_TX without propagation delay|0.299|100|0.299|29.9%|
|4|Joystick|0.33|100|0.33|0.33%|
|5|Scan keys|0.112|20|0.56|0.56%|
|6|Display|17.2|100|17.2|17.2%|
|~~7~~|~~Backcalc all features on 1 board~~|~~47.2~~|~~50~~|~~94.4~~|~~94.4%~~|
|~~8~~|~~Backcalc all features on 4 board~~|~~108.8~~|~~50~~|~~217.6~~|~~217.6%~~|
|9|Backcalc all features off 1 board|34.1|50|68.2|68.2%|
|~~10~~|~~Backcalc all features off 2 boards~~|~~46.2~~|~~50~~|~~92.4~~|~~92.4%~~|
|~~11~~|~~Backcalc all features off 4 board~~|~~70.3~~|~~50~~|~~140.6~~|~~140.6%~~|
| |Sum| | |86.63066667|87%
It can be seen from the table that when all keys of one keyboard are pressed without enabling all of its functions, the CPU consumption is considerable. Other worst case scenario considered include having all keys of one board pressed and turning on all of its functions, leading to 94.4% usage of CPU, having all keys of two boards pressed without turning all of the functions on, which requires 92.4% CPU usage, and having all keys of four boards pressed with and without activating all of the functions, both of which lead to overflow in CPU storage. It is uncommon to have all keys of the four boards or of the two boards pressed, and it is equally uncommon to have all keys of a single keyboard pressed with all of its functions activated. Therefore, it is decided that the scenario in which all keys of the single keyboard are pressed and not all functions are activated as the worst case scenario. It is important to ensure deadlines of all tasks are meet because otherwise there may be unexpected problems, such as an undetected key press or even software crash.

For the timing measurement on CAN_TX, it initially includes the propagation delay that exists during data transmission. However, after careful consideration, it is decided that once the data has been sent, the data no longer occupies CPU space, so the propagation delay is removed from the final timing measurement on CAN_TX. 












