/** @file endurance_exp_orbit.h
 *  @version 
 *  @date Feb, 2019
 *
 *  @brief
 *  Orbit a point
 *
 */

#ifndef ENDURANCE_EXP_ORBIT_H
#define ENDURANCE_EXP_ORBIT_H

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

#endif // ENDURANCE_EXP_ORBIT_H
