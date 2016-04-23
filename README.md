# Auto_trax project
This repository conatins the code for Auto_trax project by Frank Lanke Fu Tarimo, Marius Grimm, Pavel Vechersky and Marco Zorzi. We develop a badass autonomous RC car.

## IO package
The io package takes care of using ROS and interfacing with the low level hardware in order to provide topics and services to wrap the low level commands.

## Sensors package
The sensors package collects data from the sensors and convert it to more meaningful data for the control of the car.

## Dependencies
 1. The RTIMULib needs to be installed. It can be found here: https://github.com/richards-tech/RTIMULib

 2. Install the I2C tools and development libraries:

 ```
 $ sudo apt-get install i2c-tools libi2c-dev
 ```
 3. Clone the additional ros packages into your src folder:

 ```
 $ git clone git@github.com:jetsonhacks/rtimulib_ros.git
 ```

