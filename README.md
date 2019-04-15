# dji-GNC-ROS
A ROS guidance, navigation, and control stack for autonomous, ~~vision based~~ quadrotors using the following hardware:

[DJI M100](https://www.dji.com/matrice100/info#specs)

[Intel NUC 7](https://www.intel.com/content/www/us/en/products/boards-kits/nuc/kits/nuc7i7dnhe.html)

We've put vision development on hold because of sensor issues. Current nodes rely on state estimates from the DJI SDK and require GPS.

### Software Environment

Ubuntu 16.04

ROS Kinetic

[DJI Onboard SDK ROS 3.7](https://github.com/dji-sdk/Onboard-SDK-ROS)

##TODO

* Dynamic System ID
* Reintegrate vision system
