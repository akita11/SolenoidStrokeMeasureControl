# Solenoid Stroke Measure & Control

Measuring Solenoid Stroke, which corresponds to the inductance, by sensing current waveform at OFF->ON state,calculated from resistance's voltage at source of nMOSFET, amplified by non-inverting amplifier.

## How to Operate

- connect this board to M5Stck Core2's PortA.
- connect the solenoid and the power for it to this board
- generate PWM at PortA's pin3 (GPIO32). PWM frequency can be set for the solenoid used, such as 200Hz. 'PWMcontrol_Core2.m5f' is example for UIFlow.
- waveform on PortA's pin4 (GPIO33) gives current

## Measurement&Control Unit Board

see PCB/ for details, schematic and KiCAD files are available.

BOM:
- Resistor (0603) 1k(R1&R6) x 2, 10k(or 100k, for R2) x 1, 100k(R5) x 1 
- Resistor (0805) 0.2(R3)
- OPA: LM358-compatilble full swing OPA, such as [NJM2732M](https://akizukidenshi.com/catalog/g/g106018/)
- Diode: SOD123-F package, such as [SS2040FL](https://akizukidenshi.com/catalog/g/g102073/)
- Connector: ['Grove' connector](https://akizukidenshi.com/catalog/g/g112634/)
- (optional) Connector for solenoid&its power supply

## Experimental Results

Test results are in 'test240424' folder. [Takaha's CBS07300580](https://www.takaha-japan.com/product/cbs0730/) with power suppply of 12V. R5=100k&R6=10k for x101 amplification, with R4, R7, R8, R9 unused.

## Author

Junichi Akita (akita@ifdl, @akita11)
