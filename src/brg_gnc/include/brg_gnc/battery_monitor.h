/** @file battery_monitor.h
 *  @version 
 *  @date Feb, 2019
 *
 *  @brief
 *  Track battery status for endurance experiment.
 *
 */

#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

// System includes
#include "unistd.h"
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/MissionHpAction.h>
#include <dji_sdk/MissionHpUpload.h>
#include <dji_sdk/MissionHpUpdateRadius.h>
#include <dji_sdk/MissionHpUpdateYawRate.h>

// SDK core library
#include <djiosdk/dji_vehicle.hpp>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>

class BatteryMonitor
{
public:
	BatteryMonitor(int initCapacity);
	~BatteryMonitor();
	
	// callback for battery publisher
	void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
	
private:
	// parameters
	int fullCapacity;
	int limitCapacity;
	bool limitReached;
};
#endif // BATTERY_MONITOR_H
