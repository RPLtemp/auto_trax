# Auto_trax project
This repository contains the code for Auto_trax project by Frank Lanke Fu Tarimo, Marius Grimm, Pavel Vechersky and Marco Zorzi. We develop a badass autonomous RC car.

## IO package
The io package takes care of using ROS and interfacing with the low level hardware in order to provide topics and services to wrap the low level commands.

## Message package
The message package takes car of all needed custom ros message specific to the auto trax project.

## Sensors package
The sensors package collects data from the sensors and convert it to more meaningful data for the control of the car.

## Test package
The test package stores different tests on our way to fully automate auto_trax (e.g. wall following).

## Gazebo package
The gazebo package takes care of simulating auto_trax in a gazebo ros environment. 

## Dependencies
 1. Install catkin_simple
 ```
 $ cd $CATKIN_WS/src
 $ git clone git@github.com:catkin/catkin_simple.git
 $ cd $CATKIN_WS
 $ catkin_make
 ```

 2. Install RTIMULib in a directory of your choice:

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
 3. Install the following packages:

 ```
 $ sudo apt-get update
 $ sudo apt-get install i2c-tools libi2c-dev
 $ sudo apt-get install ros-indigo-ackermann-msgs
 ```
 4. Clone the additional ros packages into your workspace src folder:

 ```
 $ git clone git@github.com:jetsonhacks/rtimulib_ros.git
 ```

