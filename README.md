# Limited-Slip-Differential-Controller-for-STM32F429I
#### C, Atollic Studio, DMA, LCD

For further explanations on the decissions made, refer to the different reports (P1, P2 and P3)

## Main Features
Implemented a system to measure the rotational speed of each wheel and adjust torque distribution using analog signals.

Integrated Direct Memory Access (DMA) controllers to enhance system performance by transferring data between I/O and memory modules. 

Incorporated two accelerometers to measure longitudinal and transverse acceleration, further refining the differential control based on vehicle dynamics.

Developed a graphical user interface for real-time data visualization on the STM32F429I’s LCD screen. 

The interface displays acceleration data over time in separate windows and an X-Y diagram representing combined acceleration
