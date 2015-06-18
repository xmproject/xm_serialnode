/*
 * main.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */
#include "ros/ros.h"
#include "serial_node.h"
#include "serial_port.h"

using namespace std;
using namespace XM_SerialNode;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "XM_SerialNode");

	SerialNode serialnode(ros::NodeHandle(), ros::NodeHandle("~"));

	ros::spin();

	return 0;
}


