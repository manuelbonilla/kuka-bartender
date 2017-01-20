#ifndef BARTENDER_CONTROL__ONE_TASK_INVERSE_KINEMATICS_H
#define BARTENDER_CONTROL__ONE_TASK_INVERSE_KINEMATICS_H

#include <lwr_controllers/KinematicChainControllerBase.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <bartender_control/bartender_msg.h> 

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include <sstream>
#include "std_msgs/Float64MultiArray.h"

#include <ros/node_handle.h>
#include <ros/ros.h>
#include <time.h>

namespace bartender_control
{
	class OneTaskInverseKinematics: public controller_interface::KinematicChainControllerBase<hardware_interface::PositionJointInterface>
	{
	public:
		OneTaskInverseKinematics();
		~OneTaskInverseKinematics();

		bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		void command(const bartender_control::bartender_msg::ConstPtr &msg);

		ros::Subscriber sub_bartender_cmd;

		ros::Publisher pub_check_error;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose

		KDL::Twist x_err_;	//error position

		KDL::JntArray q_cmd_; // computed set points

		KDL::Jacobian J_;	//Jacobian

		Eigen::MatrixXd J_pinv_;	//Pseudoinverse jacobian of dynamic dimension (double)
		Eigen::Matrix<double,3,3> skew_;	//skew-matrix (3x3) of double

		struct quaternion_
		{
			KDL::Vector v;
			double a;
		} quat_curr_, quat_des_;

		KDL::Vector v_temp_;
		
		int cmd_flag_;
		
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;		// Class to calculate the jacobian of a general KDL::Chain, it is used by other solvers. It should not be used outside of KDL.
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;	// Implementation of a recursive forward position kinematics algorithm to calculate the position transformation from joint space to Cartesian space of a general kinematic chain (KDL::Chain)
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;		// Implementation of a inverse velocity kinematics algorithm based on the generalize pseudo inverse to calculate the velocity transformation from Cartesian to joint space of a general KDL::Chain. It uses a svd-calculation based on householders rotations
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;		// Implementation of a general inverse position kinematics algorithm based on Newton-Raphson iterations to calculate the position transformation from Cartesian to joint space of a general KDL::Chain. Takes joint limits into account.
	};

}

#endif
