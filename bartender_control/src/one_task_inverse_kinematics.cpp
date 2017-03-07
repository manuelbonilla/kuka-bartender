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
        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

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
        // initialization x_des_
        x_des_.p = KDL::Vector(-1, 0, 1);
        x_des_.M = KDL::Rotation::Quaternion(0, 0 , 0, -1);


        cmd_flag_ = 0;
        second_task = true;

        nh_.param<double>("alpha1", alpha1, 1);
        nh_.param<double>("alpha2", alpha2, 1);

        //Definition of publishers and subscribes

        pub_check_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 250);
        pub_check_error_gravity = nh_.advertise<std_msgs::Float64MultiArray>("error_gravity", 250);
        pub_check_initial = nh_.advertise<std_msgs::Float64MultiArray>("initial_position", 250);

        sub_bartender_cmd = nh_.subscribe("command", 250, &OneTaskInverseKinematics::command, this);

        x_err_.vel(0) = 1;
        x_err_.vel(1) = 1;
        x_err_.vel(2) = 1;

        return true;
    }

    void OneTaskInverseKinematics::command(const bartender_control::bartender_msg::ConstPtr &msg)
    {

        x_des_.p = KDL::Vector(msg->des_frame.position.x, msg->des_frame.position.y, msg->des_frame.position.z);
        x_des_.M = KDL::Rotation::Quaternion(msg->des_frame.orientation.x, msg->des_frame.orientation.y, msg->des_frame.orientation.z, msg->des_frame.orientation.w);
        
        if (!msg->arrived) cmd_flag_ = 1;
        if (msg->arrived) cmd_flag_ = 0;

    }

    //  Updating parameters for controller
    void OneTaskInverseKinematics::param_update()
    {
        // ros::spinOnce();

        // computing Jacobian
        jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);

        // computing J_pinv_
        pseudo_inverse(J_.data, J_pinv_);

        // computing forward kinematics
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

        // end-effector position error
        x_err_.vel = x_des_.p - x_.p;

        // std::cout << "X_err: " << x_err_.vel(0) << " -" << x_err_.vel(1) << " -" << x_err_.vel(2) << endl;
        // std::cout << "X_des: " << x_des_.p(0) << " -" << x_des_.p(1) << " -" << x_des_.p(2) << endl;
        // std::cout << "X_att: " << x_.p(0) << " -" <<  x_.p(1) << " -" <<  x_.p(2) << endl;

        // getting quaternion from rotation matrix
        x_.M.GetQuaternion(quat_curr_.v(0),quat_curr_.v(1),quat_curr_.v(2),quat_curr_.a);   //.M is referref to the rotation of x_. The function GetQuaternion obtains quaternion from the rotation matrix
        x_des_.M.GetQuaternion(quat_des_.v(0),quat_des_.v(1),quat_des_.v(2),quat_des_.a);

        skew_symmetric(quat_des_.v, skew_); // Gets skew-matrix from the quaternion of desired frame

        for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;                               //Initializiation of the i_element of vector v_temp
                for (int k = 0; k < skew_.cols(); k++)      
                    v_temp_(i) += skew_(i,k)*(quat_curr_.v(k)); //Sobstitution of the the i_element of vector v_temp with the product between skew_matrix and quaternion
            }

        // end-effector orientation error
        x_err_.rot = quat_curr_.a*quat_des_.v - quat_des_.a*quat_curr_.v - v_temp_;
    }

    //  Controller function:: Multy Task Inverse Kinematics
    void OneTaskInverseKinematics::update(const ros::Time& time, const ros::Duration& period)
    {
        if (second_task) nh_.param<double>("alpha1", alpha1, 4);
        else nh_.param<double>("alpha1", alpha1, 1);
        nh_.param<double>("alpha2", alpha2, 0.5);

        std_msgs::Float64MultiArray msg_error;
        std_msgs::Float64MultiArray msg_error_gravity;

        std::cout << "cmd_flag_ = " << cmd_flag_ << std::endl;

        for(int i=0; i < joint_handles_.size(); i++)
        {
            joint_msr_states_.q(i) = joint_handles_[i].getPosition();
            joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();            
        }

        bartender_control::OneTaskInverseKinematics::param_update();    //  Calculation of parameters used by controller

        //**********************************************************************************************************************//
        //  In this section you can find the MultyTask Kinematic control. The first task (alpha1) is the position control of    //
        //  the kuka-bartender to the desired position (x_des_) using the inverse kinematic control q = alpha1*pinv(J)*x_err_.  //
        //  The second task is the maximization of the potential energy of every link q = alpha2*(I-pinv(J)*J)*G, where G is    //
        //  the Gravity matrix used in the dynamic joint space model (G is a 7x1 array).                                        //
        //**********************************************************************************************************************//

        if (cmd_flag_)
        {

            //  Reinforce position task

            if (Equal(x_, x_des_, 0.2)) nh_.param<double>("alpha1", alpha2, 0.4);    //  Reinforce position task
            if (Equal(x_, x_des_, 0.15)) nh_.param<double>("alpha1", alpha2, 0.3);    //  Reinforce position task
            if (Equal(x_, x_des_, 0.125)) 
            {
                nh_.param<double>("alpha1", alpha1, 5);    //  Reinforce position task
                nh_.param<double>("alpha2", alpha2, 0.3);     //  Lower gravity task
            }

            // debug line
            
            /*std::cout << "ALPHA_1: " << alpha1 << endl;
            std::cout << "ALPHA_2: " << alpha2 << endl;*/

            //**********************************FIRST TASK***********************************************************************
            
            // computing q_dot
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += alpha1 * J_pinv_(i,k) * x_err_(k);
          
            }

            //**********************************SECOND TASK**********************************************************************

            if (second_task)
            {
                //  Null-projector (I-pinv(J)*J)
                P_null =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

                //  Creation of the second task
                q_null = alpha2 * P_null * potentialEnergy( joint_msr_states_.q );

                for (int i = 0; i < J_pinv_.rows(); i++)
                {

                    joint_des_states_.qdot(i) += alpha2 * q_null[i];

                }

            }

            //*******************************************************************************************************************

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

            for(int i=0; i < joint_handles_.size(); i++)
            {
                joint_msr_states_.q(i) = 0;
            }


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

    }

    //  Calulate the gravity vector for potential energy control (in the second taskS)
    Eigen::Matrix<double, 7, 1> OneTaskInverseKinematics::potentialEnergy(KDL::JntArray q)
    {

        KDL::JntArray G_local(7);
        id_solver_->JntToGravity(joint_msr_states_.q, G_local);

        return G_local.data ;

    }

} //  Namespace BRACKET

PLUGINLIB_EXPORT_CLASS(bartender_control::OneTaskInverseKinematics, controller_interface::ControllerBase)