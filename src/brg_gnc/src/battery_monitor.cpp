/** @file battery_monitor.cpp
 *  @version
 *  @date Feb, 2019
 *
 *  @brief Track battery status for endurance experiment.
 *
 */

#include <brg_gnc/battery_monitor.h>

// global variables
sensor_msgs::BatteryState battery_state;

BatteryMonitor(int initCapacity)
{
	// Find experiment exit condition
	limitCapacity = initCapacity - 450;
}

// callback for battery publisher
void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	battery_state = *msg;
  
	ROS_INFO("Battery Percentage:\t %d", battery_state.percentage);
};
