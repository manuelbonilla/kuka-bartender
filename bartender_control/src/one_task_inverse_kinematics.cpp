#include <bartender_control/one_task_inverse_kinematics.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>
#include "std_msgs/Bool.h"

using namespace std;

namespace bartender_control 
{
    OneTaskInverseKinematics::OneTaskInverseKinematics() {}		//costruttore	
    OneTaskInverseKinematics::~OneTaskInverseKinematics() {}	//distruttore

    bool OneTaskInverseKinematics::init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n)
    {
        if( !(KinematicChainControllerBase<hardware_interface::PositionJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("Couldn't initilize OneTaskInverseKinematics controller.");
            return false;
        }

        cout << "debug: INIT function" << endl;

        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
        ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));
    
        q_cmd_.resize(kdl_chain_.getNrOfJoints());

        J_.resize(kdl_chain_.getNrOfJoints());


        // get joint positions
        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            joint_des_states_.q(i) = joint_msr_states_.q(i);
        }

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);


        cmd_flag_ = 0;

        //Definition of publishers and subscribes

        pub_check_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 250);
        pub_check_initial = nh_.advertise<std_msgs::Float64MultiArray>("initial_position", 250);

        sub_bartender_cmd = nh_.subscribe("command", 250, &OneTaskInverseKinematics::command, this);

        nh_.param<bool>("enable_null_space", parameters_.enable_null_space , true);
        ROS_INFO_STREAM("Null Space: " << (parameters_.enable_null_space ? "Enabled" : "Disabled") );

        return true;
    }

    void OneTaskInverseKinematics::starting(const ros::Time& time)
    {
    }

    void OneTaskInverseKinematics::command(const bartender_control::bartender_msg::ConstPtr &msg)
    {

        //Reading of left message by manager
        //ROS_INFO_STREAM("MESSAGE ARRIVED");   //funziona

        x_des_.p = KDL::Vector(msg->des_frame.position.x, msg->des_frame.position.y, msg->des_frame.position.z);
        x_des_.M = KDL::Rotation::Quaternion(msg->des_frame.orientation.x, msg->des_frame.orientation.y, msg->des_frame.orientation.z, msg->des_frame.orientation.w);
        
        cmd_flag_ = 1;

    }

    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
        //cout << "debug: UPDATE function" << endl; //funziona
        // get joint positions

        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
            
        }

        if (cmd_flag_)
        {
            //cout << "debug: UPDATE function -> calcoli" << endl;  //funziona

            // computing Jacobian
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

            // computing J_pinv_
            pseudo_inverse(J_.data, J_pinv_);

            // computing forward kinematics
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

            // end-effector position error
            x_err_.vel = x_des_.p - x_.p;

            // getting quaternion from rotation matrix
            x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);	//.M is referref to the rotation of x_. The function GetQuaternion obtains quaternion from the rotation matrix
            x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);	// Gets skew-matrix from the quaternion of desired frame

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;								//Initializiation of the i_element of vector v_temp
                for (int k = 0; k < skew_.cols(); k++)		
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k));	//Sobstitution of the the i_element of vector v_temp with the product between skew_matrix and quaternion
            }

            // end-effector orientation error
            x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;

            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += J_pinv_(i,k)*x_err_(k); //removed scaling factor of .7
          
            }

            // integrating q_dot -> getting q (Euler method)
            for (int i = 0; i < joint_handles_.size(); i++)
                joint_des_states_.q(i) += period.toSec()*joint_des_states_.qdot(i);

            // joint limits saturation
            for (int i =0;  i < joint_handles_.size(); i++)
            {
                if (joint_des_states_.q(i) < joint_limits_.min(i))
                    joint_des_states_.q(i) = joint_limits_.min(i);
                if (joint_des_states_.q(i) > joint_limits_.max(i))
                    joint_des_states_.q(i) = joint_limits_.max(i);
            }

            //  New part: 
            if (parameters_.enable_null_space)
            {
                //tau_.data += N_trans_ * potentialEnergy( joint_msr_states_.q )(i);
            }

            std_msgs::Float64MultiArray msg_error;

            msg_error.data.push_back( x_err_.vel(0) );
            msg_error.data.push_back( x_err_.vel(1) );
            msg_error.data.push_back( x_err_.vel(2) );

            msg_error.data.push_back( x_err_.rot(0) );
            msg_error.data.push_back( x_err_.rot(1) );
            msg_error.data.push_back( x_err_.rot(2) );    

            pub_check_error.publish(msg_error);
             
        }
        else
        {
            cmd_flag_ = 0;

            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_initial);
            x_initial.M.GetRPY(Roll_x_init, Pitch_x_init, Yaw_x_init);
            
            std_msgs::Float64MultiArray msg_initial;

            msg_initial.data.push_back( x_initial.p(0) );
            msg_initial.data.push_back( x_initial.p(1) );
            msg_initial.data.push_back( x_initial.p(2) );

            msg_initial.data.push_back( Roll_x_init );
            msg_initial.data.push_back( Pitch_x_init );
            msg_initial.data.push_back( Yaw_x_init ); 

            pub_check_initial.publish(msg_initial);   

        }

        // set controls for joints
        for (int i = 0; i < joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommand(joint_des_states_.q(i));
        }

        /*cout << "G_local_Link_1 = " << G_local.data(0) << endl;
        cout << "G_local_Link_2 = " << G_local.data(1) << endl;
        cout << "G_local_Link_3 = " << G_local.data(2) << endl;
        cout << "G_local_Link_4 = " << G_local.data(3) << endl;
        cout << "G_local_Link_5 = " << G_local.data(4) << endl;
        cout << "G_local_Link_6 = " << G_local.data(5) << endl;
        cout << "G_local_Link_7 = " << G_local.data(6) << endl;*/
        // cout << "G_local_Link_7 = " << parameters_.gain_null_space * G_local.data << endl;
     }

    Eigen::Matrix<double, 7, 1> OneTaskInverseKinematics::potentialEnergy(KDL::JntArray q)
    {
        ROS_INFO("GRAVITY CALCULATION function");
        KDL::JntArray G_local(7);
        //id_solver_->JntToGravity(joint_msr_states_.q, G_local);

        /*cout << "G_local_Link_1 = " << G_local.data[0] << endl;
        cout << "G_local_Link_2 = " << G_local.data[1] << endl;
        cout << "G_local_Link_3 = " << G_local.data[2] << endl;
        cout << "G_local_Link_4 = " << G_local.data[3] << endl;
        cout << "G_local_Link_5 = " << G_local.data[4] << endl;
        cout << "G_local_Link_6 = " << G_local.data[5] << endl;
        cout << "G_local_Link_7 = " << G_local.data[6] << endl;
*/
        return parameters_.gain_null_space * G_local.data ;

    }
}

PLUGINLIB_EXPORT_CLASS(bartender_control::OneTaskInverseKinematics, controller_interface::ControllerBase)