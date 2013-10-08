bosch_drivers
=============

This is a fork of [bosch_drivers](http://wiki.ros.org/bosch_drivers) package [SVN repository](http://svn.code.sf.net/p/bosch-ros-pkg/code/trunk/stacks/bosch_drivers)
containing catkinized versions of the few packages we need for reading Sharp IR sensors connected to an Arduino board from
a ROS node. We use such configuration on [waiterbots](https://github.com/yujinrobot/kobuki-x/tree/hydro-devel/waiterbot),
a customized Turtlebot 2 for serving drinks.

The only catkinized packages are:

* uniserial
* arduino_interface
* bosch_drivers_common
* adc_driver

Package arduino_interface contains the Arduino firmware. to compile it and upload on board, type:

```
roscd arduino_interface/arduino_firmware
export BOARD=mega2560             # type 'make boards' to list available boards
export ARDUINO_DIR=/opt/arduino   # or wherever your arduino SDK is installed!
make
make upload
```
