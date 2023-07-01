# UFG4G89F23 Lidar Sensor and scanner

## DataSheet
[UFG4G89F23 Lidar Sensor.pdf](https://github.com/Qasim2S/Lidar-Sensor/files/11927695/UFG4G89F23.Lidar.Sensor.pdf)

## About
The UFG4G89F23 Lidar Sensor and scanner is capable of scanning data in 360 degrees to be able to create a detailed 3D visualization of the data. The design has high accuracy and real time scanning abilities. The design is easy to use and ideal for renovations of homes as the 360 degrees scan will display the room in a 3D design with dimensions.

The UFG4G89F23 Lidar Sensor is secured in a strong box with holes allowing the wires to connect to the VL53L1X Timeof-flight Sensor and the UNL2003 Stepper Motor. The VL53L1X Time-of-flight Sensor is connected to the UNL2003 Stepper Motor using a block. The UNL2003 Stepper Motor stepper motor rotates 360 degrees in 512 steps stopping at timed angles to take data, after rotating for 360 degrees, it rotates back to ensure the wires do not get tangled. The VL53L1X Time-of-flight Sensor is able to measure the distance by emitting infrared light and then collecting the light being reflected back. An equation is then used to determine the distance of the object that reflected the infrared light back.

The Lidar sensor system sends the data collected to the PC via UART, where the data is converted into cartesian points. These points are then used to create a 3D visualization by plotting the points. The whole system is controlled by the user by a press of a button

## Goal
My goal was to innovate how we measure rooms are large spaces, rather than utelizing measuring tape, this device is able to generate a scan model of a room. This technology is innovative for home application, architecture, and even archaeology due to the precise measurements and detailed 3D model generated from the measurements. 

## Components
* MSP-432E501Y Microcontroller
* VL53L1X Time-of-flight Sensor
* UNL2003 Stepper Motor
* ULN2003 Stepper Motor Driver
* Data visualization via Open3D and python

## Images
![image](https://github.com/Qasim2S/Lidar-Sensor/assets/106550804/66041e25-86b6-4e65-9346-e47a77c1ddcc)
![image](https://github.com/Qasim2S/Lidar-Sensor/assets/106550804/915c6bbf-73f9-43b6-9a55-4e8b2dd69294)
![image](https://github.com/Qasim2S/Lidar-Sensor/assets/106550804/e3c77a59-791e-4c08-84cc-f340775ebc09)




