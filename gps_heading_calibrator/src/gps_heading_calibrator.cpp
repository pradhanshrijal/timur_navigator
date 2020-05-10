#include "gps_heading_calibrator/gps_heading_calibrator.h"

namespace gps_heading_calibrator {
	gpsHeadingNode::gpsHeadingNode() :
	nodeHandle(), privateNodeHandle("~"){
		getROSParameters();
		initROSTopics();

		ros::Rate rate(rateOutput);

		int velReps = xVelTime * rateOutput;
		geometry_msgs::Twist velOut;
		velOut.linear.x = xVel;

		advertiseVel(velOut, velReps, rate);
		ros::Duration(2).sleep();
		ros::spinOnce();

		measureError();

		int revVelReps = revRepLimit * velReps;
		velOut.linear.x = -1 * revSpeedLimit * xVel;

		advertiseVel(velOut, revVelReps, rate);

		killNode();
	}

	void gpsHeadingNode::subOdomCB(const nav_msgs::Odometry::ConstPtr& odomMsg){
		yOdomDisp = odomMsg->pose.pose.position.y;
		xOdomDisp = odomMsg->pose.pose.position.x;
	}

	void gpsHeadingNode::setParamFile(std::string filePath){
		std::ofstream fileParams(filePath.c_str());

		fileParams << "navsat_transform:" << std::endl;
		fileParams << std::fixed << std::setprecision(0) << "  frequency: " << frequency << std::endl;
		fileParams << std::fixed << std::setprecision(1) << "  delay: " << delay << std::endl;
	    fileParams << std::fixed << std::setprecision(5) << "  magnetic_declination_radians: " << (mdr + headingError)<< std::endl;
	    fileParams << std::fixed << std::setprecision(5) << "  yaw_offset: " << yawOffset << std::endl;
	    fileParams << "  zero_altitude: " << std::boolalpha << zeroAltitude << std::endl;
	    fileParams << "  broadcast_utm_transform: " << std::boolalpha << broadcastUTMTransform << std::endl;
	    fileParams << "  publish_filtered_gps: " << std::boolalpha << publishFilteredGPS << std::endl;
	    fileParams << "  use_odometry_yaw: " << std::boolalpha << useOdomYaw << std::endl;
	    fileParams << "  wait_for_datum: " << std::boolalpha << waitForDatum << std::endl;

	    fileParams.close();
	}

	void gpsHeadingNode::getROSParameters(){
		// Navsat Paramters
		ros::param::get("navsat_transform/frequency", frequency);
    	ros::param::get("navsat_transform/delay", delay);
	    ros::param::get("navsat_transform/magnetic_declination_radians", mdr);
	    ros::param::get("navsat_transform/yaw_offset", yawOffset);
	    ros::param::get("navsat_transform/zero_altitude", zeroAltitude);
	    ros::param::get("navsat_transform/broadcast_utm_transform", broadcastUTMTransform);
	    ros::param::get("navsat_transform/publish_filtered_gps", publishFilteredGPS);
	    ros::param::get("navsat_transform/use_odometry_yaw", useOdomYaw);
	    ros::param::get("navsat_transform/wait_for_datum", waitForDatum);

	    // Heading Parameters
	    ros::param::get("x_vel", xVel);
	    ros::param::get("x_vel_time", xVelTime);
	    ros::param::get("rate_output", rateOutput);
	    ros::param::get("rev_speed_limit", revSpeedLimit);
	    ros::param::get("rev_rep_limit", revRepLimit);
	    ros::param::get("navsat_params_package", navsatParamsPackage);
	    ros::param::get("navsat_params_file", navsatParamsFile);

	    // Topic Names
	    ros::param::get("mb_odom_topic", mbOdomTopic);
	    ros::param::get("vel_pub_topic", velPubTopic);
	    ros::param::get("calib_status_topic", calibStatusTopic);

	    // Setting initial values
	    headingError = 0;
	}

	void gpsHeadingNode::initROSTopics(){
		subOdom = nodeHandle.subscribe(mbOdomTopic, 
								subMsgBuffer, 
								&gpsHeadingNode::subOdomCB, 
								this);
		pubVel = nodeHandle.advertise<geometry_msgs::Twist>(velPubTopic, 
								pubMsgBuffer, 
								true);
		pubCalibStatus = nodeHandle.advertise<std_msgs::Bool>(calibStatusTopic, 
								pubMsgBuffer, 
								true); 
	}

	void gpsHeadingNode::advertiseVel(geometry_msgs::Twist velPub, int velLoops, ros::Rate rosRate){
		velPub.angular.z = 0;

		for (int i = 0; i < velLoops; i++){
			pubVel.publish(velPub);
			rosRate.sleep();
		}

		velPub.linear.x = 0;
		pubVel.publish(velPub);
	}

	void gpsHeadingNode::measureError(){
		headingError = atan2(yOdomDisp, xOdomDisp);
		ROS_INFO("Error: %.1f Deg", 180/M_PI*(headingError));

		std::string path = ros::package::getPath(navsatParamsPackage) + "/param/" + navsatParamsFile;
		setParamFile(path);
	}

	void gpsHeadingNode::killNode(){
		std_msgs::Bool endNode;
		endNode.data = true;
		pubCalibStatus.publish(endNode);

		ROS_WARN("RESTART EKF NODES.");

		ros::shutdown();
	}
}