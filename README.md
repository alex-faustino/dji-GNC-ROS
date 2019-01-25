# dji-GNC-ROS
A ROS guidance, navigation, and control stack for autonomous, vision based quadrotors using the following hardware:

[DJI M100](https://www.dji.com/matrice100/info#specs)

[Intel NUC 7](https://www.intel.com/content/www/us/en/products/boards-kits/nuc/kits/nuc7i7dnhe.html)

[Optor VI sensor](https://www.robotshop.com/en/optor-visual-inertial-camera.html)

### Software Environment
Ubuntu 16.04

ROS Kinetic

[DJI Onboard SDK ROS 3.7](https://github.com/dji-sdk/Onboard-SDK-ROS)

Control and DJI SDK interface forked from [ETHZ ASL](https://github.com/ethz-asl/mav_dji_ros_interface)

VI state estimation forked from [HKUST Aerial Robotics Group](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)

Sensor driver forked from [optor-vis](https://github.com/optor-vis/optor_vi-stereo-v1)

##TODO
* Recalibrate Optor sensor
* Integrate [time sync](https://github.com/hanley6/time_sync) to fix issues with VI performance caused by Optor variable time stamping. For more info read https://github.com/ethz-asl/rovio/issues/192
* Integrate VINS-Mono with ETH SDK interface and controller
* Dynamic System ID in indoor flight arena
* Integrate GPS
* Outdoor test flights
