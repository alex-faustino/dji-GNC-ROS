/** @file optimal_alt_exp.cpp
 *  @version
 *  @date Oct, 2019
 *
 *  @brief
 *  Fly straight line trajectories in to and out of the wind
 *
 */

#include <brg_gnc/optimal_alt_exp.h>

using namespace DJI::OSDK;

// global variables
ros::ServiceClient     		hotpoint_upload_service;
ros::ServiceClient     		hotpoint_action_service;
ros::ServiceClient     		hotpoint_update_yawRate_Service;
ros::ServiceClient     		hotpoint_updateRadius_service;
ros::ServiceClient     		drone_activation_service;
ros::ServiceClient     		sdk_ctrl_authority_service;
ros::ServiceClient     		drone_task_service;
sensor_msgs::NavSatFix 		gps_pos;
ros::Subscriber        		gps_pos_subscriber;
sensor_msgs::BatteryState 	battery_state;
ros::Subscriber		   		battery_state_subscriber;
float INIT_SOC;
float BASE_SOC;
float CURRENT_SOC;
float CUTOFF;
bool COLLECT_DATA = false;
FILE *batteryDataFile;
FILE *gpsPosDataFile;

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	gps_pos = *msg;
	
	if (COLLECT_DATA) 
	{
		// Print time, lat, and long to text file
		fprintf(gpsPosDataFile,"%f\t%f\t%f\t%f\n",
				gps_pos.header.stamp.toSec(),
				gps_pos.latitude,
				gps_pos.longitude,
				gps_pos.altitude);
	}
}

void batteryStateCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{	
	battery_state = *msg;
  
	CURRENT_SOC = battery_state.percentage;

	if ((CURRENT_SOC == INIT_SOC) || (INIT_SOC == NAN))
	{
		return;
	} 
	else if (((INIT_SOC - CURRENT_SOC) >= 5.0) && (!COLLECT_DATA)) 
	{
		BASE_SOC = CURRENT_SOC;
		ROS_INFO("Baseline battery capacity set: %f", BASE_SOC);
		COLLECT_DATA = true;
	} 
	else 
	{
		if ((BASE_SOC - CURRENT_SOC) >= CUTOFF)
		{
			COLLECT_DATA = false;
			fclose(gpsPosDataFile);
			endHotpointMission();
		}
	}

	if (COLLECT_DATA) 
	{
		// Print time, lat, and long to text file
		fprintf(batteryDataFile,"%f\t%f\n",
				battery_state.header.stamp.toSec(),
				CURRENT_SOC);
	}
}


bool runHotpointMission(int initialRadius,
						int initialAlt,
					    float initialAngularSpeed,
				        int responseTimeout)
{
	ros::spinOnce();

	// Hotpoint Mission: Create hotpoint
	dji_sdk::MissionHotpointTask hotpointTask;
	setHotPointInit(hotpointTask, initialRadius, initialAngularSpeed, initialAlt);

	// Hotpoint Mission: Initialize
	initHotpointMission(hotpointTask);

	// Takeoff
	if (takeoff().result)
	{
		ROS_INFO("Takeoff command sent successfully");
	}
	else
	{
		ROS_WARN("Failed sending takeoff command");
		return false;
	}
	ros::Duration(15).sleep();

	// Start
	ROS_INFO("Starting orbit mission");
	if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                      DJI::OSDK::MISSION_ACTION::START).result)
	{
		ROS_INFO("Mission start command sent successfully");
	}
	else
	{
		ROS_WARN("Failed sending mission start command");
		return false;
	}

	INIT_SOC = CURRENT_SOC;
	ROS_INFO("Initial battery capacity set: %f", INIT_SOC);

	return true;
}

bool endHotpointMission()
{
	// Stop
	ROS_INFO("Ending orbit mission");
	if (missionAction(DJI::OSDK::DJI_MISSION_TYPE::HOTPOINT,
                      DJI::OSDK::MISSION_ACTION::STOP).result)
	{
		ROS_INFO("Mission stop command sent successfully");
	}
	else
	{
		ROS_WARN("Failed sending mission stop command");
		return false;
	}
	
	ROS_INFO("Returning home");
	if (goHome().result)
	{
		ROS_INFO("Return home command sent successfully");
	}
	else
	{
		ROS_WARN("Failed sending return home command");
		return false;
	}
	
	ros::Duration(30).sleep();
	
	ROS_INFO("Landing");
	if (land().result)
	{
		ROS_INFO("Land command sent successfully");
	}
	else
	{
		ROS_WARN("Failed sending land command");
		return false;
	}
	
	fclose(batteryDataFile);
	fclose(gpsPosDataFile);
}

void setHotPointInit(dji_sdk::MissionHotpointTask& hotpointTask,
					 int initialRadius,
					 float initialAngularSpeed,
					 int initialAlt)
{
	hotpointTask.latitude      = gps_pos.latitude;
	hotpointTask.longitude     = gps_pos.longitude;
	hotpointTask.altitude      = initialAlt;
	hotpointTask.radius        = initialRadius;
	hotpointTask.angular_speed = initialAngularSpeed;
	hotpointTask.is_clockwise  = 0;
	hotpointTask.start_point   = 0;
	hotpointTask.yaw_mode      = 0;
}

ServiceAck
initHotpointMission(dji_sdk::MissionHotpointTask& hotpointTask)
{
	dji_sdk::MissionHpUpload missionHpUpload;
	missionHpUpload.request.hotpoint_task = hotpointTask;
	hotpoint_upload_service.call(missionHpUpload);
	return ServiceAck(
			missionHpUpload.response.result,
			missionHpUpload.response.cmd_set,
			missionHpUpload.response.cmd_id,
			missionHpUpload.response.ack_data);
}

ServiceAck
missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
              DJI::OSDK::MISSION_ACTION   action)
{
	dji_sdk::MissionHpAction missionHpAction;
  
	missionHpAction.request.action = action;
	hotpoint_action_service.call(missionHpAction);
	if (!missionHpAction.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 missionHpAction.response.cmd_set,
				 missionHpAction.response.cmd_id);
		ROS_WARN("ack.data: %i", missionHpAction.response.ack_data);
	}
	return ServiceAck(
			missionHpAction.response.result,
			missionHpAction.response.cmd_set,
			missionHpAction.response.cmd_id,
			missionHpAction.response.ack_data);
}

ServiceAck
activate()
{
	dji_sdk::Activation activation;
	drone_activation_service.call(activation);
	if (!activation.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 activation.response.cmd_set,
				 activation.response.cmd_id);
		ROS_WARN("ack.data: %i", activation.response.ack_data);
	}
	return ServiceAck(
			activation.response.result, activation.response.cmd_set,
			activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck
obtainCtrlAuthority()
{
	dji_sdk::SDKControlAuthority sdkAuthority;
	sdkAuthority.request.control_enable = 1;
	sdk_ctrl_authority_service.call(sdkAuthority);
	if (!sdkAuthority.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 sdkAuthority.response.cmd_set,
				 sdkAuthority.response.cmd_id);
		ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
	}
	return ServiceAck(
			sdkAuthority.response.result, sdkAuthority.response.cmd_set,
            sdkAuthority.response.cmd_id,
            sdkAuthority.response.ack_data);
}

ServiceAck
takeoff()
{
	dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = 4;
	drone_task_service.call(droneTaskControl);
	if (!droneTaskControl.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 droneTaskControl.response.cmd_set,
				 droneTaskControl.response.cmd_id);
		ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
	}
	return ServiceAck(
			droneTaskControl.response.result,
			droneTaskControl.response.cmd_set,
			droneTaskControl.response.cmd_id,
			droneTaskControl.response.ack_data);
}

ServiceAck
goHome()
{
	dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = 1;
	drone_task_service.call(droneTaskControl);
	if (!droneTaskControl.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 droneTaskControl.response.cmd_set,
                 droneTaskControl.response.cmd_id);
		ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
	}
	return ServiceAck(
			droneTaskControl.response.result,
			droneTaskControl.response.cmd_set,
			droneTaskControl.response.cmd_id,
			droneTaskControl.response.ack_data);
}

ServiceAck
land()
{
	dji_sdk::DroneTaskControl droneTaskControl;
	droneTaskControl.request.task = 6;
	drone_task_service.call(droneTaskControl);
	if (!droneTaskControl.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 droneTaskControl.response.cmd_set,
				 droneTaskControl.response.cmd_id);
		ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
	}
	return ServiceAck(
			droneTaskControl.response.result,
			droneTaskControl.response.cmd_set,
			droneTaskControl.response.cmd_id,
			droneTaskControl.response.ack_data);
}

ServiceAck
hotpointUpdateRadius(float radius)
{
	dji_sdk::MissionHpUpdateRadius missionHpUpdateRadius;
	missionHpUpdateRadius.request.radius = radius;
	hotpoint_updateRadius_service.call(missionHpUpdateRadius);
	if (!missionHpUpdateRadius.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 missionHpUpdateRadius.response.cmd_set,
				 missionHpUpdateRadius.response.cmd_id);
		ROS_WARN("ack.data: %i",
				 missionHpUpdateRadius.response.ack_data);
	}
	return ServiceAck(missionHpUpdateRadius.response.result,
					  missionHpUpdateRadius.response.cmd_set,
                      missionHpUpdateRadius.response.cmd_id,
                      missionHpUpdateRadius.response.ack_data);
}

ServiceAck
hotpointUpdateYawRate(float yawRate, int direction)
{
	dji_sdk::MissionHpUpdateYawRate missionHpUpdateYawRate;
	missionHpUpdateYawRate.request.yaw_rate  = yawRate;
	missionHpUpdateYawRate.request.direction = direction;
	hotpoint_update_yawRate_Service.call(missionHpUpdateYawRate);
	if (!missionHpUpdateYawRate.response.result)
	{
		ROS_WARN("ack.info: set = %i id = %i",
				 missionHpUpdateYawRate.response.cmd_set,
				 missionHpUpdateYawRate.response.cmd_id);
		ROS_WARN("ack.data: %i",
				 missionHpUpdateYawRate.response.ack_data);
	}
	return ServiceAck(missionHpUpdateYawRate.response.result,
					  missionHpUpdateYawRate.response.cmd_set,
                      missionHpUpdateYawRate.response.cmd_id,
                      missionHpUpdateYawRate.response.ack_data);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "sdk_demo_mission");
	ros::NodeHandle nh;
	
	// Setup variables for use
	uint8_t wayptPolygonSides;
	int     initRadius, initAlt;
	float	initLinVelocity;
	float	initAngularSpeed;
	int     responseTimeout = 1;
	int		b, g, trialNum;
	char batteryDataFileName [64];
	char gpsPosDataFileName [64];
	
	// Display interactive prompt get experiment parameters
	std::cout << "Enter orbit parameters" << std::endl;
	std::cout << "Orbit radius in meters: ";
    std::cin >> initRadius;
    
	std::cout << "Enter linear velocity in meters per second: ";
	std::cin >> initLinVelocity;

	std::cout << "Enter altitude in meters: ";
	std::cin >> initAlt;
	
	std::cout << "Enter battery parameters" << std::endl;
	std::cout << "Enter cutoff battery SoC in \%: ";
	std::cin >> CUTOFF;

	std::cout << "Enter trial number for these parameters: ";
	std::cin >> trialNum;

	// Create data files to write to
	b = sprintf(batteryDataFileName, "%dm-%dmps-%dpercent-battery-%d.txt",
				initRadius, int(initLinVelocity), int(CUTOFF), trialNum);
	batteryDataFile = fopen(batteryDataFileName, "w");
	
	
	g = sprintf(gpsPosDataFileName, "%dm-%dmps-%dpercent-gps-pos-%d.txt",
				initRadius, int(initLinVelocity), int(CUTOFF), trialNum);
	gpsPosDataFile = fopen(gpsPosDataFileName, "w");
	
    // Create battery monitor
    // BatteryMonitor BatteryMonitor(INIT_CAPACITY);

	// ROS stuff
	hotpoint_upload_service = 
		nh.serviceClient<dji_sdk::MissionHpUpload>(
			"dji_sdk/mission_hotpoint_upload");
	hotpoint_action_service = 
		nh.serviceClient<dji_sdk::MissionHpAction>(
			"dji_sdk/mission_hotpoint_action");
	hotpoint_updateRadius_service =
		nh.serviceClient<dji_sdk::MissionHpUpdateRadius>(
			"dji_sdk/mission_hotpoint_updateRadius");
	hotpoint_update_yawRate_Service =
		nh.serviceClient<dji_sdk::MissionHpUpdateYawRate>(
			"dji_sdk/mission_hotpoint_updateYawRate");
	drone_activation_service =
		nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
	sdk_ctrl_authority_service = 
		nh.serviceClient<dji_sdk::SDKControlAuthority>(
			"dji_sdk/sdk_control_authority");
	drone_task_service =
		nh.serviceClient<dji_sdk::DroneTaskControl>(
			"dji_sdk/drone_task_control");
	gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
		"dji_sdk/gps_position", 10, &gpsPosCallback);
	battery_state_subscriber = nh.subscribe<sensor_msgs::BatteryState>(
		"dji_sdk/battery_state", 10, &batteryStateCallback);

	// Activate
	if (activate().result)
	{
		ROS_INFO("Activated successfully");
	}
	else
	{
		ROS_WARN("Failed activation");
		return -1;
	}

	// Obtain Control Authority
	ServiceAck ack = obtainCtrlAuthority();
	if (ack.result)
	{
		ROS_INFO("Obtain SDK control Authority successfully");
	}
	else
	{
		if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0)
		{
			ROS_INFO("Obtain SDK control Authority in progess, "
					 "send the cmd again");
			obtainCtrlAuthority();
		}
		else
		{
			ROS_WARN("Failed Obtain SDK control Authority");
			return -1;

		}
	}
	
	// Calculate angular rate in degrees per second
	initAngularSpeed = (initLinVelocity/initRadius) * (180.0/C_PI);
	
    // Hotpoint call
    runHotpointMission(initRadius, initAlt, initAngularSpeed, responseTimeout);
	
	ros::spin();
	
  return 0;
}
