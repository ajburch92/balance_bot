## Balance Bot Beaglebone Black

This repository contains the supporting files for Austin Burch and Zongyao Jin's Texas A&M University MEEN 652 Multivariable Control System Design Project. This repo includes:
- a standalone balance bot program
- a ROS package developed in conjunction with the open source pid package. This dependent package can be found at: http://wiki.ros.org/pid
- Linux scripts required for the beaglebone cape and pinmux setup.

![p-v-gpid](/images/balance_bot.png)

## [ROS Package: `balance_bot_for_pidpkg`](./balance_bot_for_pidpkg)

This package was built for use with the ros pid open source package. This package gives the beaglebone black the capabilties to tune and visualize any balancing bot online with a remote interface using rqt. 

- [`launch files`](/balance_bot_for_pidpkg/launch) - the balance_bot.launch file will launch all the required nodes on the beaglebone black except the driving node. Currently, this node needs to be launched seperatedly, and is named balance_bot_node_for_pidpkg. You can preconfigure the controller settings within this file. The balance_bot_QT.launch file starts the nodes required for rqt visualizations and pid dynamic reconfiguring.
- [`src`](/balance_bot_for_pidpkg/src) - The src folder contains all the cpp files required to launch the nodes described above. 

## [Standalone Program: `balance_bot_program`](./balance_bot_program)

Most of the code is revised based on Derek Molloy's libraries avaliable at Exploring BeagleBone Black website

--------------------

util.*

which does file operation (read/write)

--------------------

GPIO.*

which is able to activate a GPIO pin, set direction, set HIGH/LOW, set active voltage level, get the pin number it's controller, and also write or read value while keeping the file stream open in order to increase performance

--------------------

PWM.* 

which does the interface with PWM pins, it is able to set or get period, duty cycle, duty cycle percentage, polarity in varies ways, and also read and write while keep the file stream open to increase speed

--------------------

eqep.*

which does the interface with BBB's EQEP system, it is able to read quadrature rotary encoder counts in the raw form, radian form, and degree form, also initialize the rotary encoder

---------------------

I2CDevice.*

which does the interface with the I2C bus, it is able to open the I2C bus, and read or write registers

---------------------

MPU9250.*

which is the child class of I2CDevice, and it does the interface with MPU9250 IMU. It is able to configure the IMU, read or write accelerometer/gyro data from the IMU, and also calculate roll and pitch angle of the IMU based on the gravity vectors and displaying data to user

---------------------

L298N.*

which does the interface with L298N motor driver, it is able to activate a motor by specifying a PWM channel controlling the magnitude and 2 GPIO pins controlling the directions. It is able to drive a DC motor by a user specified voltage between -12V ~ +12V

---------------------

[`workedController.cpp`](balance_bot_program/workedController.cpp)

which is the main control loop that balances a cart-pole robot. Explanation of this code is in the code itself.

## [Linux Files: `scripts`](./scripts)
- [`deploy_balance_bot_cape`](/scripts/deploy_balance_bot_cape) - beaglebone black cape
- [`uEnv.txt`](/scripts/uEnv.txt) - /boot/uEnv.txt file needed for pinmux
- [`balance_bot_cape.dts`](/source/balance_bot_cape.dts) - overlay file for beaglebone black cape

