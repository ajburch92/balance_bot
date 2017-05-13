/*
 by: Austin Burch
 date: 4/29/17
 license: MIT, happy coding
 
 
 
 You may use this code however you like.
 
 */


//this file receives control effort and maps to voltage

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <unistd.h>
#include <ratio>
#include <iomanip>

#include "MPU9250.h"
#include "L298N.h"
#include "eqep.h"

#define BALANCING_SPS 100

using namespace ZJ;

namespace control_effort_namespace
{
  static double control_effort = 0.0;
}
using namespace control_effort_namespace;

void ControlEffortCallback(const std_msgs::Float64& control_effort_input)
{
	control_effort = control_effort_input.data;
	//ROS_INFO=("control_effort: %f",control_effort);
	printf("control_effort: %f",control_effort);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "balance_node_for_pidpkg"); //FIX ERRORS HERE !!!
  ros::NodeHandle balance_node_for_pidpkg;

  std_msgs::Float64 plant_state;
  ros::Publisher balance_pub = balance_node_for_pidpkg.advertise<std_msgs::Float64>("state", 1);

  ros::Subscriber sub = balance_node_for_pidpkg.subscribe("control_effort", 1, ControlEffortCallback);

  int i = 0;
  double theta = 0.0;
  double u = 0.0;
  ros::Rate loop_rate(BALANCING_SPS);

  //create IMU and motor driving objects
  //create struct instances

          // motor/pwm init
	L298N motorL("pwm2a", 27, 65);
	L298N motorR("pwm1a", 66, 67);

	    // imu init
	MPU9250 imu(2, 0x68);
	    // imu config
	usleep(10000);
	imu.setAccRange(MPU9250::PLUSMINUS_16_G);
	imu.setGyroRange(MPU9250::PLUSMINUS_2000_DPS);
	usleep(10000);

	float dTheta = 0;
	float phi = 0;
	float thetaAccum = 0;
	float thetaAdjusted = 0;
	float thetaGyro = 0.0f;
	float thetaComp = 0.0f;

	while(ros::ok())
	{
	
		ros::spinOnce();
		//sense
		imu.readSensor();
		
		thetaAdjusted = 0.001*thetaAccum;
		theta = imu.getAccRoll() * 0.017453292519943f - thetaAdjusted+0.05;
		dTheta = imu.getGyroOmegaX() * 0.017453292519943f;
		thetaAccum += theta;
   	        thetaGyro = thetaComp + imu.getGyroOmegaX() * 0.017453292519943f * 1.0f/BALANCING_SPS;
		thetaComp = 0.95*thetaGyro + 0.05*theta;

		u = -1.0*control_effort;

		if (u > 0.001) {
			motorL.runByVoltage(u+1.1);// add stall voltage
			motorR.runByVoltage(u+1.1);
		} else if (u<0.001) {
			motorL.runByVoltage(u-1.1);
			motorR.runByVoltage(u-1.1);
		}

 		theta = (double)thetaComp;
	        plant_state.data = theta;

 		balance_pub.publish(plant_state);
  		i++;
   		ROS_INFO("state: %f  control_effor: %f", plant_state.data, u);
  		loop_rate.sleep();
 	}

  return 0;
}

