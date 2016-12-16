#ifndef BARTENDER_MANAGER_H
#define BARTENDER_MANAGER_H

#include <sensor_msgs/JointState.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <kdl/kdl.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <bartender_control/bartender_msg.h> 
#include <Eigen/LU>
//#include "bartender_manager/BartenderManagerSrv.h"

class BartenderManager {

	public:
	    BartenderManager();
	    ~BartenderManager();

	    //void buildBaseMvMap();
	    //void buildCompositeMvMap();

		//void checkCallback(const std_msgs::Bool & check_msg);
		//void TrunkCallback(const geometry_msgs::Pose & trunk_msg);

		//bool parseSrvAction(anymal_manager::AnymalManagerSrv::Request &req, anymal_manager::AnymalManagerSrv::Response &res);
		//void doAction(int action_type, std::string action, int to_move);
		std::vector<float> doFollow(float x, float y, float z, bool init_trajectory);
		//std::vector<anymal_control::anymal_msg> buildType1action(std::string action, int id);
		//std::vector<anymal_control::anymal_msg> buildType8action(std::string action);
		//std::vector<anymal_control::anymal_msg> buildRobotRot(int angle);
		//std::vector<anymal_control::anymal_msg> buildWalk(std::string action);
		void EulerToQuaternion(float R, float P, float Y);
		//std::vector<anymal_control::anymal_msg> pending_jobs_;
		void InsertCartPosEE();
	
	private:
		ros::NodeHandle n_;
		ros::Subscriber	sub_bartender_check_;
		//ros::Subscriber	sub_anymal_trunk_position_;

		ros::Publisher pub_bartender_cmd_;
		//ros::ServiceServer anymal_manager_srv_;

		//std::map<std::string, geometry_msgs::Pose> available_base_mv_;

		KDL::Frame x_;
		KDL::Frame x_des_;
		geometry_msgs::Pose pose_rot_;

		//bool trunk_semaphore_;
		//int n_rem_jobs_;
		//int call_count_trunk_;
		//int trunk_rot_angle_;
	};





#endif
