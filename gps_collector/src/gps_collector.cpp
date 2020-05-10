#include "gps_collector/gps_collector.h"

namespace gps_collector{
	gpsCollectorNode::gpsCollectorNode() :
	nodeHandle(), privateNodeHandle("~")
	{
		getROSParams();
		initParams();
		getFilePath();
		initROSTopics();
		userInstructions();
	}

	void gpsCollectorNode::joyCB(const sensor_msgs::Joy::ConstPtr& joyMsg){
		ros::Duration minDuration(1.0);

		if(joyMsg->buttons[collectorButtonNumber] == 1){
			if (storageFile.is_open()){
				currentTime = ros::Time::now();

				if((currentTime - previousTime) > minDuration){
					double latDiff = abs((latCurrent - latPrev)*pow(10,6))*pow(10,-6);
					double lonDiff = abs((lonCurrent - lonPrev)*pow(10,6))*pow(10,-6);

					if((latDiff > minGPSDiff)||(lonDiff > minGPSDiff)){
						ROS_INFO("Ahoy!!! Matey!!");
						userInstructions();
						wpCounter++;
						storageFile << std::fixed << std::setprecision(8) << latCurrent << " " << lonCurrent << std::endl;
						latPrev = latCurrent;
						lonPrev = lonCurrent;
					} else {
						ROS_WARN("Waypoints are too close!!!");
					}

					previousTime = currentTime;
				}
			} else {
				ROS_ERROR("File is not Open....");
			}
		}

		if(joyMsg->buttons[endButtonNumber] == 1){
			killNode();
		}
	}

	void gpsCollectorNode::gpsCB(const sensor_msgs::NavSatFix::ConstPtr& gpsMsg){
		latCurrent = gpsMsg->latitude;
		lonCurrent = gpsMsg->longitude;
	}

	void gpsCollectorNode::killNode(){
		std_msgs::Bool endNode;
		endNode.data = true;
		pubCollectorStatus.publish(endNode);

		ros::shutdown();
	}

	void gpsCollectorNode::getROSParams(){
		ros::param::get("collect_but_num", collectorButtonNumber);
		ros::param::get("collect_but_sym", collectorButtonSymbol);
		ros::param::get("end_but_num", endButtonNumber);
		ros::param::get("end_but_sym", endButtonSymbol);

		ros::param::get("wp_package", wpPackage);
		ros::param::get("wp_file", wpFilePath);

		ros::param::get("joy_topic", joyTopic);
		ros::param::get("gps_topic", gpsTopic);
		ros::param::get("collec_status_topic", collectorStatusTopic);
	}

	void gpsCollectorNode::initParams(){
		wpCounter = 0;
		latCurrent = 0;
		lonCurrent = 0;
		latPrev = 0;
		lonPrev = 0;
	}

	void gpsCollectorNode::getFilePath(){
		totalPath = ros::package::getPath(wpPackage) + wpFilePath;
		storageFile.open (totalPath.c_str());
		ROS_INFO("The GPS files are stored in: %s", totalPath.c_str());
	}

	void gpsCollectorNode::initROSTopics(){
		subJoy = nodeHandle.subscribe(joyTopic, 
								subMsgBuffer, 
								&gpsCollectorNode::joyCB, 
								this);

		subGPS = nodeHandle.subscribe(gpsTopic, 
								subMsgBuffer, 
								&gpsCollectorNode::gpsCB, 
								this);

		ROS_INFO("GPS Collector Active!!");

		pubCollectorStatus = nodeHandle.advertise<std_msgs::Bool>(collectorStatusTopic, 
											pubMsgBuffer, 
											true);
	}

	void gpsCollectorNode::userInstructions(){
		std::cout << std::endl;
		std::cout << "Press " << collectorButtonSymbol.c_str() << " button for collection." << std::endl;
		std::cout << "Press " << endButtonSymbol.c_str() << " button to end." << std::endl;
		std::cout << std::endl;
	}
}