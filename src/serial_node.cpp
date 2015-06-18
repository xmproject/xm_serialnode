/*
 * serial_node.cpp
 *
 *  Created on: 2012-4-8
 *      Author: startar
 */

#include "serial_node.h"

namespace XM_SerialNode {

SerialNode::SerialNode(const ros::NodeHandle &node, const ros::NodeHandle &prinode):
		m_node(node), m_privateNode(prinode)
{
	cout << "SerialNode Object created!" << endl;
	loadParams();			// 载入参数

	// 创建SerialPort对象
	m_pSerialPort = make_shared<SerialPort>();
	m_pSerialPort->setSerialParams(m_serialParams);
	m_pSerialPort->setTimeOut(m_timeOut);
	m_pSerialPort->setCallbackFunc(bind(&SerialNode::serialCallback, this, _1));

	// 订阅两个串口数据Topic
	m_sub_xmmsg = m_node.subscribe("SendSerialData", 1000,
			&SerialNode::XM_DataGram_Callback, this);
	m_sub_raw = m_node.subscribe("SendSerialData_raw", 1000,
			&SerialNode::XM_RawData_Callback, this);

	// 公布一个Topic, 发布从串口中读到的数据报文
	m_pub_Allrecv = m_node.advertise<xm_msgs::xm_DataGram>("AllRecvData", 1000);

	// 启动串口线程
	m_pSerialPort->startThread();

}

SerialNode::~SerialNode()
{
	m_pSerialPort->stopThread();
}

void SerialNode::loadParams()
{
	m_serialParams.serialPort 	= "/dev/ttyUSB0";
	m_serialParams.baudRate 	= 921600;
	m_serialParams.flowControl	= 0;
	m_serialParams.parity		= 0;
	m_serialParams.stopBits		= 0;
	m_timeOut = 100;

	m_privateNode.getParam("serialPort", m_serialParams.serialPort);
	m_privateNode.getParam("baudRate", (int&)(m_serialParams.baudRate));
	m_privateNode.getParam("flowControl", (int&)(m_serialParams.flowControl));
	m_privateNode.getParam("parity", (int&)(m_serialParams.parity));
	m_privateNode.getParam("stopBits", (int&)(m_serialParams.stopBits));
	m_privateNode.getParam("timeout", m_timeOut);
}

void SerialNode::XM_DataGram_Callback(const xm_msgs::xm_DataGram::ConstPtr &msg)
{
	cout << "Sending new datagram !" << endl;
	m_pSerialPort->writeDataGram(*msg);
}

void SerialNode::XM_RawData_Callback(const xm_msgs::xm_Uint8Array::ConstPtr &msg)
{
	m_pSerialPort->writeRaw(msg->data);
}

void SerialNode::serialCallback(xm_msgs::xm_DataGramPtr pDatagram)
{
	// 找到与receiverID对应的ROS::Publisher
	ros::Publisher &pub = m_pubsByRecvID[pDatagram->receiver];
	if (!pub)		// 如果这个Publisher还不存在, 就创建一个
	{
		stringstream ss;
		ss << "RecvData/" << (int)(pDatagram->receiver);					// 生成topic名字
		pub = m_node.advertise<xm_msgs::xm_DataGram>(ss.str(), 1000);		// advertise之
	}
	pub.publish(pDatagram);

	// 向AllRecvData中发布一个msg
	m_pub_Allrecv.publish(pDatagram);
}

} /* namespace XM_SerialNode */
