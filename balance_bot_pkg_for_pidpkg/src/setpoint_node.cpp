/*
 by: Austin Burch
 date: 4/23/17
 license: MIT, happy coding
 
 
 
 You may use this code however you like.
 
 */

// this file publishes the setpoint

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>
#include "eqep.h"
#include "util.h"

#define SETPOINT_SPS 1

using namespace ZJ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "balance_setpoint_node");
  ROS_INFO("Starting balance_setpoint_node publisher");
  ros::NodeHandle balance_setpoint_node;

  std_msgs::Float64 setpoint;
  setpoint.data = 0.0;

//   namespace control_effort_namespace
// {
//   static double right_wheel_control_effort = 0.0;
//   static double left_wheel_control_effort = 0.0;
// }
// using namespace control_effort_namespace;

// void RightWheelControlEffortCallback(const std_msgs::Float64& right_wheel_control_effort_input)
// {
//   right_wheel_control_effort = right_wheel_control_effort_input.data
// }

// void LeftWheelControlEffortCallback(const std_msgs::Float64& left_wheel_control_effort_input)
// {
//   left_wheel_control_effort = left_wheel_control_effort_input.data
// }

// //
//  ros::Publisher servo_state_pub = left_wheel.advertise<std_msgs::Float64>("state", 1);
//  ros::Publisher servo_state_pub = right_wheel.advertise<std_msgs::Float64>("state", 1);

//  ros::Subscriber sub = left_wheel.subscribe("control_effort", 1, RightWheelControlEffortCallback);
//  ros::Subscriber sub = right_wheel.subscribe("control_effort", 1, RightWheelControlEffortCallback);

  //ros::Publisher setpoint_pub = setpoint_node.advertise<std_msgs::Float64>("right_wheel_setpoint", 1);

  ros::Publisher setpoint_pub = balance_setpoint_node.advertise<std_msgs::Float64>("setpoint", 1);

  ros::Rate loop_rate(SETPOINT_SPS);   // change setpoint every 5 seconds
    
  double setpoint_val = 0.0;
  double accum = 0.0;
  double pos = 0.0;
  //eqep objects

  //eqep init and struct 
    //init eqep channels
    EQEP eqepL(1);
    //EQEP eqepR(0);

  while (ros::ok())
  {
    ros::spinOnce();
    pos = (double)eqepL.getRad();

    //adjust setpoint (PID object?)
//    if (pos >=  3.14f || pos <= -3.14f) { 
    	setpoint_val = 0.0;
//	eqepL.setZero();
//    } 
//    if (pos >= 2.0f) {
//	setpoint_val = (double)-2.0f;  
//    } else if (pos< -2.0f) {
//    	setpoint_val = (double)2.0f;
//    }
    
    setpoint.data = setpoint_val;
      
    setpoint_pub.publish(setpoint);     // publish twice so graph gets it as a step
    setpoint.data = 0 - setpoint.data;
    setpoint_pub.publish(setpoint);
    ROS_INFO("setpoint_val = %f",setpoint_val);

    loop_rate.sleep();
  }
}
