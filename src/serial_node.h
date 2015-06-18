/*
 * serial_node.h
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#ifndef SERIALNODE_H_
#define SERIALNODE_H_

#include <ros/ros.h>
#include <map>
#include "serial_port.h"
#include "xm_msgs/xm_DataGram.h"
#include "xm_msgs/xm_Uint8Array.h"

namespace XM_SerialNode {

class SerialNode {
private:
	SerialParams 		m_serialParams; 		// 串口的配置数据
	int 				m_timeOut; 				// 数据报超时时间

	shared_ptr<SerialPort>	m_pSerialPort;

	ros::NodeHandle 	m_node, m_privateNode;	// 当前NodeHandle与私有NodeHandle

	// Note: 按照ROS的文档, ROS提供的对象统统是thread-safe的, 所以不用管互斥了
	ros::Subscriber		m_sub_xmmsg;			// SendSerialData的ROS订阅对象
	ros::Subscriber		m_sub_raw;				// SendSerialData_raw的ROS订阅对象
	ros::Publisher		m_pub_Allrecv;			// AllRecvData的ROS发布对象
	std::map<uint8_t, ros::Publisher>	m_pubsByRecvID;		// 各receiveID对应的ROS发布对象

	void loadParams();

	void XM_DataGram_Callback(const xm_msgs::xm_DataGram::ConstPtr &msg);
	void XM_RawData_Callback(const xm_msgs::xm_Uint8Array::ConstPtr &msg);

	void serialCallback(xm_msgs::xm_DataGramPtr pDatagram);

public:
	SerialNode(const ros::NodeHandle &node, const ros::NodeHandle &prinode);
	virtual ~SerialNode();
};

} /* namespace XM_SerialNode */
#endif /* SERIALNODE_H_ */
