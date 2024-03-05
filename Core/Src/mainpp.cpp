/*
 * mainpp.cpp
 *
 *  Created on: Mar 1, 2024
 *      Author: sanji
 */
#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle nh;
std_msgs::String str_msg;

char hello[] = "Hello world from STM32!";
//extern int16_t sensor_buff[5];
ros::Publisher chatter("chatter", &str_msg);

void setup(void){
	nh.initNode();
	nh.advertise(chatter);
}

void loop(void){

	str_msg.data = hello;
	chatter.publish(&str_msg);
	nh.spinOnce();
//	HAL_Delay(1);
//	    Sensor_data.data_length =5;
//	    	Sensor_data.data= sensor_buff;
//	    	sensor.publish(&Sensor_data);
//	    	nh.spinOnce();
}
