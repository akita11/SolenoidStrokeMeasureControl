# Solenoid Stroke Measure & Control

Measuring Solenoid Stroke, which corresponds to the inductance, by sensing current waveform at OFF->ON state,calculated from resistance's voltage at source of nMOSFET, amplified by non-inverting amplifier.

## How to Use

- connect this board to M5Stck Core2's PortA.
- connect the solenoid and the power for it to this board

### for SolenoidStrokelUnit:

Using [SolenoidStrokeUNIT](https://github.com/akita11/SolenoidStrokeUNIT), trained models are in Control_ESP32/src or Control_Python. Technical details are [paper1](https://arxiv.org/abs/2405.11721) and [paper2](https://arxiv.org/abs/2411.07270)

- Measure_... for parameter measurement
- Control... for position control (using M5Stack or Python)


### for SolenoidMeasureControlUnit:

- generate PWM at PortA's pin3 (GPIO32). PWM frequency can be set for the solenoid used, such as 100Hz. 
- waveform on PortA's pin4 (GPIO33) gives current


## Measurement&Control Unit Board

available at [https://www.switch-science.com/products/10174].

(old: SolenoidMeasureControlUnit) available at [here](https://github.com/akita11/SolenoidStrokeMeasureControlUNIT) or [purchase here](https://www.switch-science.com/products/9717). 

## Experimental Results

(old: SolenoidMeasureControlUnit)

Test results are in 'test240424' folder. [Takaha's CBS07300580](https://www.takaha-japan.com/product/cbs0730/) with power suppply of 12V. R5=100k&R6=10k for x101 amplification, with R4, R7, R8, R9 unused.


## Author

Junichi Akita (akita@ifdl, @akita11)
