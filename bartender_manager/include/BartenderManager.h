#ifndef BARTENDER_MANAGER_H
#define BARTENDER_MANAGER_H

#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <bartender_control/bartender_msg.h> 
#include <Eigen/LU>
#include <XmlRpcValue.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"


class BartenderManager {

	public:
	    BartenderManager();
	    ~BartenderManager();

	    void checkCallback_right(const std_msgs::Float64MultiArray &msg_err);
	    void checkCallback_left(const std_msgs::Float64MultiArray &msg_err);
	    void checkCallback_right_initial(const std_msgs::Float64MultiArray &msg_init);
	    void checkCallback_left_initial(const std_msgs::Float64MultiArray &msg_init);
		double *EulerToQuaternion(float R, float P, float Y);
		void DrinkSelection();
		void Publish();
		void Init();
		void Grasping();
		void Pouring();
		void InitialPosition();

		bool BottleGrasping = false;
		bool ActionPouring = false;
		bool Init_cond = false;

		bartender_control::bartender_msg msg_right;
		bartender_control::bartender_msg msg_left;

		KDL::Frame x_err_right;
		KDL::Frame x_err_left;

		KDL::Frame x_err_compare;

		KDL::Frame x_right_initial;
		KDL::Frame x_left_initial;
		
	private:
		ros::NodeHandle n_;

		ros::Publisher pub_bartender_cmd_right;
		ros::Publisher pub_bartender_cmd_left;

		ros::Subscriber sub_bartender_err_right;
		ros::Subscriber sub_bartender_err_left;

		ros::Subscriber sub_bartender_init_right;
		ros::Subscriber sub_bartender_init_left;

		KDL::Frame x_;

		KDL::Frame x_bottle;

		geometry_msgs::Pose pose_rot_;

		std::map<std::string,KDL::Frame > bottle;	//Positions array: the first fild is the name (STRING), second is the position (VECTOR) 

		float roll_bottle, pitch_bottle, yaw_bottle;
		double *q_bottle, *q_err_right, *q_err_left, *q_init_right, *q_init_left;

	};

#endif
