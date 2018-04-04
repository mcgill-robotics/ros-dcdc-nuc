# ROS DCDC NUC driver

[status]: https://dev.mcgillrobotics.com/buildStatus/icon?job=ros-dcdc-nuc/master
[url]: https://dev.mcgillrobotics.com/job/ros-dcdc-nuc/job/master
[![status]][url]

This is a driver for the Mini-Box DCDC NUC computer power suppy.

The point of this application is to have status information of the power
supply in ROS.

## Setting up

You must clone this repository as `dcdc-nuc` into your catkin workspace:
```bash
git clone https://github.com/mcgill-robotics/ros-dcdc-nuc dcdc_nuc
```

You will need to update the firmware for the DCDC NUC to be able to communicate
with linux. Earlier version of the firmware has a bug that cause it to fail to
emulate under linux.

Refer to the [DCDC NUC wiki page](http://wiki.mini-box.com/index.php?title=DCDC-NUC#Bootloader_Mode)
for more detail.

## Dependencies

This package requires `libusb-dev` to build. You can get it by running on:

```bash
sudo apt-get install libusb-dev
```
## Running

To run, simply launch the `dcdc_nuc_node` node as such:

```bash
rosrun dcdc_nuc dcdc_nuc_node
```
