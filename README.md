# LIDAR-Spatial-Mapping
This is a spatial measurement system project taht utilizes the the time-offlight sensor to acquire information about the area around it. The system utilizes a stepper motor
to provide a 360-degree measurement of distance within a single vertical geometric plane (y-z),
for every 360-degree measurement to produce a 3D visualization of the space that was mapped.

The system contains a microcontroller, a stepper motor, and a time-of-flight sensor. The
MSP432E401Y microcontroller oversees and controls the entire embedded system, delivers
power from the PC to the other components, and sends the measured data to the PC via USB. The
mounted ToF sensor has a 360-degree range of motion on one axis, allowing it to record distance
measurements in a single vertical plane, due to the stepper motor.

The VL53L1X ToF sensor generates infrared light pulses and measures the amount of
time the pulse is "in-flight" before being detected to estimate the distance to objects. The
distance is calculated by translating the analog value to digital and processing it in conjunction
with the sensor's configuration settings. This information is sent to the microcontroller through
I2C.

The system connects to a PC and runs the python scripts that are included. The
microcontroller sends status messages and distance measurements to the PC through UART. The
data is then visualized using a Python script that is included

**Device Overview**

Embedded Spatial Measurement System:
- 3D visualization of data using Python and open3D
- Data transmission with PC via USB
- Measures 360-degree distance within single plane

Texas Instruments MSP432E401Y Microcontroller:

- Arm Cortex-M4F Central Processing Unit
- 12 MHz clock speed
- Programmed in C (Alternatively Assembly or C++)

VL53L1X Time-of-Flight (ToF) Sensor:

- 2.6V - 3.5V operating voltage
- Up to 4m range
- 20 mm range error

ULN2003 Stepper Motor Driver:
- 5V - 12V operating voltage
- 512 steps per 360 degrees rotation
- LED step state indicators

Data communication:
- I2C serial communication between the microcontroller and ToF sensor
- UART serial communication between the microcontroller and PC

Visualization:
- Supported on Python 3.8.5
- Utilizes Open3D python library
- 3D visualization of mapped data
