/** @file optimal_alt_exp.h
 *  @version 
 *  @date Oct, 2019
 *
 *  @brief
 *  Demonstrate effectiveness of altitude controller for reducing power consumption
 *
 */

#ifndef OPTIMAL_ALT_EXP_H
#define OPTIMAL_ALT_EXP_H

// System includes
#include "unistd.h"
#include <iostream>
#include <stdio.h>

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

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

typedef struct ServiceAck
{
	bool         result;
	int          cmd_set;
	int          cmd_id;
	unsigned int ack_data;
	ServiceAck(bool res, int set, int id, unsigned int ack)
		: result(res), cmd_set(set), cmd_id(id), ack_data(ack)
	{
	}
	ServiceAck()
	{
	}
} ServiceAck;

bool runHotpointMission(int initialRadius,
						float initialAngularSpeed,
						int responseTimeout);
						
bool endHotpointMission();

void setHotPointInit(dji_sdk::MissionHotpointTask& hotpointTask,
					 int initialRadius,
					 float initialAngularSpeed,
					 int initialAlt);

ServiceAck initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask);

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION   action);

ServiceAck activate();

ServiceAck obtainCtrlAuthority();

ServiceAck takeoff();

ServiceAck goHome();

ServiceAck land();

ServiceAck hotpointUpdateRadius(float radius);

ServiceAck hotpointUpdateYawRate(float yawRate, int direction);

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

#endif // OPTIMAL_ALT_EXP_H
