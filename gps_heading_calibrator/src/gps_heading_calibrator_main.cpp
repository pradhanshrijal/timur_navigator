#include "gps_heading_calibrator/gps_heading_calibrator.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "gps_heading_calibrator");
	gps_heading_calibrator::gpsHeadingNode node;
	ros::spin();
	return 0;
}