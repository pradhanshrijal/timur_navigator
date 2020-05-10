#include "gps_collector/gps_collector.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "gps_collector");
	gps_collector::gpsCollectorNode node;
	ros::spin();
	return 0;
}