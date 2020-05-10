#include "gps_follower/gps_follower.h"

namespace gps_follower{
	gpsFollowerNode::gpsFollowerNode() :
	nodeHandle(), privateNodeHandle("~"){
		getROSParams();
		initParams();
		initROSTopics();

		ROS_INFO("Follower Active!!!");

		getWaypoints();
		setWaypoints();
		setGPSTargets();
	}

	void gpsFollowerNode::killNode(){
		std_msgs::Bool endNode;
		endNode.data = true;
		pubFollowerStatus.publish(endNode);

		ros::shutdown();
	}

	void gpsFollowerNode::mbServerFailure() {
		counterWait++;
		if(counterWait > 3) {
			ROS_ERROR("Failed move_base server.... beg_waypoints_gps_nav terminated!!");
            killNode();
		}
	}

	geometry_msgs::PointStamped gpsFollowerNode::convertFixToUTM(const double latMsg, 
																	const double lonMsg){
		double xUTM = 0, 
				yUTM = 0;

		geometry_msgs::PointStamped poseUTM = geometry_msgs::PointStamped();

		RobotLocalization::NavsatConversions::LLtoUTM(latMsg, lonMsg, xUTM, yUTM, zoneUTM);

		poseUTM.header.frame_id = utmFrame;
		poseUTM.header.stamp = ros::Time(0);
		poseUTM.point.x = xUTM;
		poseUTM.point.y = yUTM;
		poseUTM.point.z = 0;

		return poseUTM;
	}

	geometry_msgs::PointStamped gpsFollowerNode::transformUTMToOdom(geometry_msgs::PointStamped utmMsg){
		geometry_msgs::PointStamped poseMap;
		bool followTarget = true;
		tf::TransformListener tfListen;
		ros::Time currentTime = ros::Time::now();

		while (followTarget) {
			try {
				utmMsg.header.stamp = ros::Time::now();
				tfListen.waitForTransform(odomFrame, utmFrame, currentTime, ros::Duration(3.0));
				tfListen.transformPoint(odomFrame, utmMsg, poseMap);
				followTarget = false;
			} catch (tf::TransformException& ex) {
				ROS_WARN("%s", ex.what());
				ros::Duration(0.01).sleep();
			}
		}

		return poseMap;
	}

	move_base_msgs::MoveBaseGoal gpsFollowerNode::buildGoal(geometry_msgs::PointStamped goalMsg, 
															geometry_msgs::PointStamped nextMsg, 
															bool endPoint){
		move_base_msgs::MoveBaseGoal mbGoal;

		mbGoal.target_pose.header.frame_id = odomFrame;
		mbGoal.target_pose.header.stamp = ros::Time::now();
		mbGoal.target_pose.pose.position.x = goalMsg.point.x;
		mbGoal.target_pose.pose.position.y = goalMsg.point.y;

		if (endPoint == false) {
			tf::Matrix3x3 eulerAngle;
			tf::Quaternion quatAngle;

			float xGoal = goalMsg.point.x,
					yGoal = goalMsg.point.y,
					xNext = nextMsg.point.x,
					yNext = nextMsg.point.y,
					xDiff = xNext - xGoal,
					yDiff = yNext - yGoal;

			float roll = 0,
					pitch = 0,
					yaw = 0;

			// Conversion to Quaternion		
			yaw = atan2(yDiff, xDiff);
			eulerAngle.setEulerYPR(yaw, pitch, roll);
			eulerAngle.getRotation(quatAngle);

			mbGoal.target_pose.pose.orientation.x = quatAngle.getX();
			mbGoal.target_pose.pose.orientation.y = quatAngle.getY();
			mbGoal.target_pose.pose.orientation.z = quatAngle.getZ();
			mbGoal.target_pose.pose.orientation.w = quatAngle.getW();
		} else {
			mbGoal.target_pose.pose.orientation.w = 1.0;
		}

		return mbGoal;
	}

	void gpsFollowerNode::getROSParams(){
		ros::param::get("wp_package", wpPackage);
		ros::param::get("wp_file", wpPath);
		ros::param::get("follow_status_topic", followStatusTopic);
		ros::param::get("mb_topic", mbTopic);
		ros::param::get("utm_frame", utmFrame);
		ros::param::get("odom_frame", odomFrame);
	}

	void gpsFollowerNode::initParams(){
		totalPath = ros::package::getPath(wpPackage) + wpPath;

		counter = 0;
		counterWp = 0;
		counterWait = 0;
		wpNum = 0;

		goalLat = 0;
		goalLon = 0;
		nextLat = 0;
		nextLon = 0;
	}

	void gpsFollowerNode::initROSTopics(){
		pubFollowerStatus = nodeHandle.advertise<std_msgs::Bool>(followStatusTopic, pubBoolMsgBuffer, true);
	}

	void gpsFollowerNode::getWaypoints(){
		fileLoader.open (totalPath.c_str());
 
		if (fileLoader.is_open()){
			while(!fileLoader.eof()){
				++counter;
			}
			counter -= 1;
			wpNum = counter / 2;
			ROS_INFO("%.0f GPS Waypoints Received!!", wpNum);
			fileLoader.close();
		} else {
			ROS_ERROR("ERROR!!!.... Waypoint file not open!!!");
			killNode();
		}
	}

	void gpsFollowerNode::setWaypoints(){ // Can it be joint with getWaypoints?
		double lat = 0, 
				lon = 0;
		fileLoader.open (totalPath.c_str());

		for (int i = 0; i < wpNum; i++){
			fileLoader >> lat;
			fileLoader >> lon;
			vectorWp.push_back(std::make_pair(lat, lon));
		}

		fileLoader.close();
	}

	void gpsFollowerNode::setGPSTargets(){
		moveBaseClient mbClient(mbTopic, true);

		while(!mbClient.waitForServer(ros::Duration(9.0))){
			mbServerFailure();
			ROS_INFO("Wait for move_base action server!!");
		}

		for (iterater = vectorWp.begin(); iterater < vectorWp.end(); iterater++){
			bool lastTarget = false;

			goalLat = iterater->first;
			goalLon = iterater->second;

			if (iterater < (vectorWp.end() - 1)){
				iterater++;
				nextLat = iterater->first;
				nextLon = iterater->second;
				iterater--;
			} else {
				nextLat = iterater->first; // Basically put the same value
				nextLon = iterater->second;
				lastTarget = true;
			}

			ROS_INFO("Target Lat: %.8f  Target Lon: %.8f", goalLat, goalLon);

			// Conversion of Fix to UTM
			goalUTM = convertFixToUTM(goalLat, goalLon);
			nextUTM = convertFixToUTM(nextLat, nextLon);

			// Conversion of UTM to Odom
			goalMap = transformUTMToOdom(goalUTM);
			nextMap = transformUTMToOdom(nextUTM);

			ROS_INFO("x: %.8f  y: %.8f", nextMap.point.x, nextMap.point.y);

			move_base_msgs::MoveBaseGoal goalMb = buildGoal(goalMap, nextMap, lastTarget);

			ROS_INFO("Follow Goal!!");
			mbClient.sendGoal(goalMb);
			mbClient.waitForResult();

			if (mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				ROS_INFO("Goal Reached!!!");
				ROS_INFO("----------------------------------");
			} else {
				ROS_ERROR("Goal Unreachable!!!");
				ROS_INFO("Node Terminated!!!");
				killNode();
			}
		}

		ROS_INFO("All Goals are reached!!!\n");
		ROS_INFO("Termination!!!");
		killNode();
	}
}