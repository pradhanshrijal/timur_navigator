#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <math.h>
#include <stdlib.h>
#include <utility>

#include <fstream>

namespace gps_heading_calibrator{
	constexpr uint8_t subMsgBuffer = 100;
	constexpr uint8_t pubMsgBuffer = 100;
	
	class gpsHeadingNode {
	public:
		gpsHeadingNode();
		~gpsHeadingNode(){};

	private:
		// Node Handles
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privateNodeHandle;

		// Subscribers and Publishers
		ros::Subscriber subOdom;
		ros::Publisher pubVel;
		ros::Publisher pubCalibStatus;

		// ROS Callbacks
		void subOdomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);

		// Functions
		void setParamFile(std::string filePath);
		
		void getROSParameters();
		void initROSTopics();
		void advertiseVel(geometry_msgs::Twist velPub, int velLoops, ros::Rate rosRate);
		void measureError();
		void killNode();

		// Variables
		float frequency, 
				delay, 
				mdr, 
				yawOffset, 
				xVel, 
				xVelTime, 
				rateOutput, 
				revSpeedLimit, 
				revRepLimit,
				yOdomDisp,
				xOdomDisp;

		bool zeroAltitude, 
				broadcastUTMTransform, 
				publishFilteredGPS, 
				useOdomYaw, 
				waitForDatum; 

		std::string navsatParamsFile,
					navsatParamsPackage,
					mbOdomTopic,
					velPubTopic,
					calibStatusTopic;

		double headingError {};
	};
}