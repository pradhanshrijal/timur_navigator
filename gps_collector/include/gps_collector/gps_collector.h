#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <stdexcept>
#include <math.h>
#include <utility>
#include <fstream>
#include <iostream>

namespace gps_collector{
	constexpr uint8_t subMsgBuffer = 100;
	constexpr uint8_t pubMsgBuffer = 100;
	constexpr double minGPSDiff = 10 * pow(10, -6);

	class gpsCollectorNode{
	public:
		gpsCollectorNode();
		~gpsCollectorNode(){};

	private:
		// Node Handles
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privateNodeHandle;

		// Publisher and Subscriber
		ros::Subscriber subJoy;
		ros::Subscriber subGPS;
		ros::Publisher pubCollectorStatus;

		// ROS Variables
		ros::Time previousTime {},
					currentTime {};

		// Callbacks
		void joyCB(const sensor_msgs::Joy::ConstPtr& joyMsg);
		void gpsCB(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg);

		// Functions
		void killNode();

		void getROSParams();
		void initParams();
		void getFilePath();
		void initROSTopics();
		void userInstructions();

		// Variables
		int collectorButtonNumber,
			endButtonNumber;

		std::string collectorButtonSymbol,
					endButtonSymbol,
					wpPackage,
					wpFilePath,
					joyTopic,
					gpsTopic,
					collectorStatusTopic,
					totalPath {};

		double wpCounter,
				latCurrent,
				lonCurrent,
				latPrev,
				lonPrev;

		std::ofstream storageFile;
	};
}