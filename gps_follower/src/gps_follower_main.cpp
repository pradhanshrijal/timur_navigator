#include "gps_follower/gps_follower.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "gps_follower");
	gps_follower::gpsFollowerNode node;
	ros::spin();
	return 0;
}