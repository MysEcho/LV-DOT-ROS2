/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node (ROS2)
*/
#include <rclcpp/rclcpp.hpp>
#include <onboard_detector/dynamicDetector.h>

int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	auto detector = std::make_shared<onboardDetector::dynamicDetector>();

	rclcpp::spin(detector);
	rclcpp::shutdown();

	return 0;
}
