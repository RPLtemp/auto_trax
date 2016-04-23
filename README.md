# Auto_trax project
This repository conatins the code for Auto_trax project by Frank Lanke Fu Tarimo, Marius Grimm, Pavel Vechersky and Marco Zorzi. We develop a badass autonomous RC car.

## IO package
The io package takes care of using ROS and interfacing with the low level hardware in order to provide topics and services to wrap the low level commands.

## Sensors package
The sensors package collects data from the sensors and convert it to more meaningful data for the control of the car.

## Dependencies
 1. Install RTIMULib in a directory of your choice:

 ```
 $ git clone git@github.com:richards-tech/RTIMULib.git
 ```
 Go to the directory where RTIMULib was cloned and enter:

 ```
 $ cd RTIMULib
 $ mkdir build
 $ cd build
 $ cmake ..
 $ make -j4
 $ sudo make install
 $ sudo ldconfig
 ```
 2. Install the following packages:

 ```
 $ sudo apt-get update
 $ sudo apt-get install i2c-tools libi2c-dev
 $ sudo apt-get install ros-indigo-ackermann-msgs
 ```
 3. Clone the additional ros packages into your workspace src folder:

 ```
 $ git clone git@github.com:jetsonhacks/rtimulib_ros.git
 ```

