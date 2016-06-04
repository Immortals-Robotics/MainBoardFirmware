# Immortals Firmware
Immortals robot firmware for the Robocup Soccer Small-Size League. The firmware of the FPGA is developed using Altium Designer. It allows developing both logic design and embedded software for the soft processor in one tool. The embedded software is written in C, and the logic design is done using a combination of schematics, Verilog and VHDL.

Features that require parallelism and least delay are implemented in logic, including:
* Rotary encoder sensor reading
* Voltage booster control signal generation
* BLDC commutation
* Timing of power mosfets’ switching to avoid short circuit
* An alternative simplified control loop for motors, based solely on hall sensors, used when rotary encoder data is somehow unavailable. These situations include:
  * There was an error reading the encoder
  * The encoder is not installed mechanically, on electronically, so they’re disabled by the user

The following features are implemented inside the TSK3000 processor:
* An error detection system, similar to the ones used in modern cars’ ECUs. It detects many errors, including low-voltage situations, voltage spikes, short circuits, over-temperature,  daughter boards’ malfunction, packet drops, sensor reading errors, motor stuck situations and much more. Then these errors are written to the flash memory for later diagnostics. But some of them are categorized as critical, and the processor halts the function of the malfunctioned part, and report the situation by making an error-specific sound, turning the STOP led on, and sending it over to the main computer. These errors should be cleared by using the diagnostics software, and the person erasing it should solve the problem before clearing it.
* Acting as an odometer, calculating the usage of each part, including BLDC motors, voltage booster unit, servo motor, rotary encoders and the wheels. These data then are used by the rest of the team, to know when to change or check each unit.
* Providing real-time diagnostics data over wireless link, such as each motor speed, voltages of some critical points, state of debugging switches (believe it or not, they are the mother of many other malfunctions!), IMU data and voltage booster state.
* PID control loop with torque converter for each motor. The control loop frequency is 1.2 kHz, and delay time is 122 uS. The loop is so time-critical, and so is written in pure assembly.
