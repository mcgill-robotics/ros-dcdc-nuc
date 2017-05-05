# ROS DCDC NUC driver

This is a driver for the Mini-Box DCDC NUC computer power suppy.

The point of this application is to have status information of the power 
supply in ROS.

## Setting up

You must clone this repository as `dcdc-nuc` into your catkin workspace:
```bash
git clone https://github.com/mcgill-robotics/ros-dcdc-nuc dcdc_nuc
```

You will need to patch your HID kernel driver to be able to communicate with 
the power supply. The McGill Robotics `compsys` repository contains the script
to patch the kernel.

If you do not have the `compsys` repository cloned, you can do so by running:

```bash
git clone https://github.com/mcgill-robotics/compsys.git
```

To run the script, change directory into compsys repository and run:

```bash
./drivers/dcdc-nuc/install
```

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
