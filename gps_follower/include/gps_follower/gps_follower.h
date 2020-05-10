#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <utility>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_localization/navsat_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <math.h>

namespace gps_follower{
	constexpr uint8_t pubBoolMsgBuffer = 100;

	class gpsFollowerNode{
	public:
		gpsFollowerNode();
		~gpsFollowerNode(){};
	private:
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privateNodeHandle;

		// Publisher
		ros::Publisher pubFollowerStatus;

		// Functions
		void killNode();
		void mbServerFailure();
		geometry_msgs::PointStamped convertFixToUTM(const double latMsg, 
													const double lonMsg);
		geometry_msgs::PointStamped transformUTMToOdom(geometry_msgs::PointStamped utmMsg);
		move_base_msgs::MoveBaseGoal buildGoal(geometry_msgs::PointStamped goalMsg, 
												geometry_msgs::PointStamped nextMsg, 
												bool endPoint);

		void getROSParams();
		void initParams();
		void initROSTopics();
		void getWaypoints();
		void setWaypoints();
		void setGPSTargets();

		// Variables
		std::string wpPackage,
					wpPath,
					followStatusTopic,
					mbTopic,
					utmFrame,
					odomFrame,
					totalPath,
					zoneUTM {};

		int counter,
			counterWp,
			counterWait;

		double wpNum,
				goalLat,
				goalLon,
				nextLat,
				nextLon;

		std::ifstream fileLoader {};

		std::vector <std::pair<double, double>> vectorWp {};
		std::vector <std::pair<double, double>> ::iterator iterater {};

		typedef actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> moveBaseClient;

		geometry_msgs::PointStamped goalUTM {},
									nextUTM {},
									goalMap {},
									nextMap {};
	};
}