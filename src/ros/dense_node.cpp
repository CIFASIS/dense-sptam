#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dense");

	nodelet::Loader nodelet;
	nodelet::M_string remap(ros::names::getRemappings());
	nodelet::V_string nargv;
	std::string nodelet_name = ros::this_node::getName();
	nodelet.load(nodelet_name, "dense/denseNodelet", remap, nargv);

	ros::MultiThreadedSpinner spinner;
	spinner.spin();

	return 0;
}
