#include "BartenderManager.h"

using namespace std;

BartenderManager::BartenderManager() 
{
   
    pub_bartender_cmd_right = n_.advertise<bartender_control::bartender_msg>("/right_arm/bartender_control/command", 250);
    pub_bartender_cmd_left = n_.advertise<bartender_control::bartender_msg>("/left_arm/bartender_control/command", 250);

    sub_bartender_err_right = n_.subscribe("/right_arm/bartender_control/error", 250, &BartenderManager::checkCallback_right, this);
    sub_bartender_err_left = n_.subscribe("/left_arm/bartender_control/error", 250, &BartenderManager::checkCallback_left, this);

    sub_bartender_init_right = n_.subscribe("/right_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_right_initial, this);
    sub_bartender_init_left = n_.subscribe("/left_arm/bartender_control/initial_position", 250, &BartenderManager::checkCallback_left_initial, this);

    x_err_compare.p(0) = 0;
    x_err_compare.p(1) = 0;
    x_err_compare.p(2) = 0;
    x_err_compare = KDL::Frame(KDL::Rotation::Quaternion(1, 0, 0, 0), x_err_compare.p);

    /*x_err_right.p(0) = 1;
    x_err_right.p(1) = 1;
    x_err_right.p(2) = 1;

    x_err_left.p(0) = 1;
    x_err_left.p(1) = 1;
    x_err_left.p(2) = 1;*/
}

BartenderManager::~BartenderManager() {}

//Function callback for right arm
void BartenderManager::checkCallback_right(const std_msgs::Float64MultiArray & msg_err) {
    
    x_err_right.p(0) = msg_err.data[0];
    x_err_right.p(1) = msg_err.data[1];
    x_err_right.p(2) = msg_err.data[2];

    q_err_right = BartenderManager::EulerToQuaternion(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

    x_err_right = KDL::Frame(KDL::Rotation::Quaternion(q_err_right[0], q_err_right[1], q_err_right[2], q_err_right[3]), x_err_right.p);
    
}

//Function callback for left arm
void BartenderManager::checkCallback_left(const std_msgs::Float64MultiArray & msg_err) {
    
    x_err_left.p(0) = msg_err.data[0];
    x_err_left.p(1) = msg_err.data[1];
    x_err_left.p(2) = msg_err.data[2];

    q_err_left = BartenderManager::EulerToQuaternion(msg_err.data[3], msg_err.data[4], msg_err.data[5]);

    x_err_left = KDL::Frame(KDL::Rotation::Quaternion(q_err_left[0], q_err_left[1], q_err_left[2], q_err_left[3]), x_err_left.p);

}

//Function callback for right arm initial position
void BartenderManager::checkCallback_right_initial(const std_msgs::Float64MultiArray &msg_init) {
    
    x_right_initial.p(0) = msg_init.data[0];
    x_right_initial.p(1) = msg_init.data[1];
    x_right_initial.p(2) = msg_init.data[2];

    q_init_right = BartenderManager::EulerToQuaternion(msg_init.data[3], msg_init.data[4], msg_init.data[5]);

    x_right_initial = KDL::Frame(KDL::Rotation::Quaternion(q_init_right[0], q_init_right[1], q_init_right[2], q_init_right[3]), x_right_initial.p);
    
}

//Function callback for left arm initial position
void BartenderManager::checkCallback_left_initial(const std_msgs::Float64MultiArray &msg_init) {
    
    x_left_initial.p(0) = msg_init.data[0];
    x_left_initial.p(1) = msg_init.data[1];
    x_left_initial.p(2) = msg_init.data[2];

    q_init_left = BartenderManager::EulerToQuaternion(msg_init.data[3], msg_init.data[4], msg_init.data[5]);

    x_left_initial = KDL::Frame(KDL::Rotation::Quaternion(q_init_left[0], q_init_left[1], q_init_left[2], q_init_left[3]), x_left_initial.p);
    
}

//This function initializes the bottle map (string,frame)
void BartenderManager::Init ()
{

	x_bottle.p(0) = -0.8;
	x_bottle.p(1) = 0.3;
	x_bottle.p(2) = 0.2;

	roll_bottle = 0;
	pitch_bottle = -90;
	yaw_bottle = 0;

	q_bottle = BartenderManager::EulerToQuaternion(roll_bottle, pitch_bottle, yaw_bottle);

	x_bottle = KDL::Frame(KDL::Rotation::Quaternion(q_bottle[0], q_bottle[1], q_bottle[2], q_bottle[3]), x_bottle.p);

	bottle["vodka"] = x_bottle;

	x_bottle.p(1) = 0;
	bottle["gin"] = x_bottle;

	x_bottle.p(1) = -0.3;
	bottle["lemon"] = x_bottle;

	x_bottle.p(0) = -0.6;
	x_bottle.p(1) = 0;
	x_bottle.p(2) = 0.1;
	bottle["glass"] = x_bottle;

}

//Function who transforms Euler agles (RPY) in quaternion
double *BartenderManager::EulerToQuaternion(float R, float P, float Y)
{
	
	static double q_[4];
	double t0 = std::cos((3.142 * Y/ 180)*0.5f);
	double t1 = std::sin((3.142 * Y/ 180)*0.5f);
	double t2 = std::cos((3.142 * R/ 180)*0.5f);
	double t3 = std::sin((3.142 * R/ 180)*0.5f);
	double t4 = std::cos((3.142 * P/ 180)*0.5f);
	double t5 = std::sin((3.142 * P/ 180)*0.5f);

	q_[0] = t0 * t2 * t4 + t1 * t3 * t5;
	q_[1] = t0 * t3 * t4 - t1 * t2 * t5;
	q_[2] = t0 * t2 * t5 + t1 * t3 * t4;
	q_[3] = t1 * t2 * t4 - t0 * t3 * t5;

	return q_;

}

//Function who let the user insert 2 bottle and it creates 2 msg
void BartenderManager::DrinkSelection ()
{	
	string choise1, choise2;

  	cout << "Please, enter the first bottle (vodka, gin, lemon): " << endl;
  	getline (cin, choise1);

  	cout << "Please, enter the second bottle (vodka, gin, lemon): " << endl;
  	getline (cin, choise2);

  	cout << "You have chosen " << choise1 << " and " << choise2 << endl;

  	for (auto bot : bottle){
  		if(!choise1.compare(bot.first)){
  			msg_right.des_frame.position.x = bot.second.p(0);
		    msg_right.des_frame.position.y = bot.second.p(1);
		    msg_right.des_frame.position.z = bot.second.p(2);

		    //Put desired EE orientation in the control message
		    msg_right.des_frame.orientation.x = q_bottle[0];
		    msg_right.des_frame.orientation.y = q_bottle[1];
		    msg_right.des_frame.orientation.z = q_bottle[2];
		    msg_right.des_frame.orientation.w = q_bottle[3];

		    msg_right.arrived = false;
		 }
  	}

  	for (auto bot : bottle){
  		if(!choise2.compare(bot.first)){
  			msg_left.des_frame.position.x = bot.second.p(0);
		    msg_left.des_frame.position.y = bot.second.p(1);
		    msg_left.des_frame.position.z = bot.second.p(2);

		    //Put desired EE orientation in the control message
		    msg_left.des_frame.orientation.x = q_bottle[0];
		    msg_left.des_frame.orientation.y = q_bottle[1];
		    msg_left.des_frame.orientation.z = q_bottle[2];
		    msg_left.des_frame.orientation.w = q_bottle[3];

		    msg_left.arrived = false;
  		}
  	}

}

void BartenderManager::Grasping()
{
	msg_right.arrived = false;
	msg_right.des_frame.position.x = -0.6;
	msg_right.des_frame.position.y = 0.1;
	msg_right.des_frame.position.z = 0.2;

	msg_left.arrived = false;
	msg_left.des_frame.position.x = -0.6;
	msg_left.des_frame.position.y = -0.1;
	msg_left.des_frame.position.z = 0.2;

}

void BartenderManager::Pouring()
{

}

void BartenderManager::InitialPosition()
{
	msg_right.des_frame.position.x = x_right_initial.p(0);
	msg_right.des_frame.position.y = x_right_initial.p(1);
	msg_right.des_frame.position.z = x_right_initial.p(2);

	/*msg_right.des_frame.orientation.x = q_bottle[0];
	msg_right.des_frame.orientation.y = q_bottle[1];
	msg_right.des_frame.orientation.z = q_bottle[2];
	msg_right.des_frame.orientation.w = q_bottle[3];*/

	msg_left.des_frame.position.x = x_left_initial.p(0);
	msg_left.des_frame.position.y = x_left_initial.p(1);
	msg_left.des_frame.position.z = x_left_initial.p(2);

	/*msg_left.des_frame.orientation.x = q_bottle[0];
	msg_left.des_frame.orientation.y = q_bottle[1];
	msg_left.des_frame.orientation.z = q_bottle[2];
	msg_left.des_frame.orientation.w = q_bottle[3];*/

}

//Function who pubblishes 2 msg
void BartenderManager::Publish() 
{
	//cout << "debug: PUBLISH function" << endl;	//funziona

	pub_bartender_cmd_right.publish(msg_right);
	pub_bartender_cmd_left.publish(msg_left);
	ros::spinOnce();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bartender_manager");

	BartenderManager manager;

	manager.Init();

	while (ros::ok())
	{
		int action = 1;
		manager.DrinkSelection();
			
		while (action !=4)
		{	
			switch (action)
			{

				case(1):
					ROS_INFO("FIRST action");
					//manager.Publish();
					while (Equal(manager.x_err_right, manager.x_err_compare, 0.05) && Equal(manager.x_err_left, manager.x_err_compare, 0.05) && action == 1)
	        		{	
	        			manager.Grasping();
			    		//manager.Publish();
			    		manager.x_err_right.p(0) = 1;
			    		action = 2;
			    		ROS_INFO("Both arm on targets, no i'm going to grasping and pouring in the glass");
			        }
					break;

				case(2):
					ROS_INFO("SECOND action");
					//manager.Grasping();
					//manager.Publish();
					while (Equal(manager.x_err_right, manager.x_err_compare, 0.05) && Equal(manager.x_err_left, manager.x_err_compare, 0.05) && action == 2)
	        		{	
			        	manager.InitialPosition();
			    		//manager.Publish();
			    		manager.x_err_right.p(0) = 1;
			    		action = 3;
			    		ROS_INFO("Now it's time to pouring!!! ");
			        }
					break;

				case(3):
					ROS_INFO("THIRD action");
					//manager.InitialPosition();
					//manager.Publish();
					while (Equal(manager.x_err_right, manager.x_err_compare, 0.05) && Equal(manager.x_err_left, manager.x_err_compare, 0.05) && action == 3)
	        		{	
			        	//manager.InitialPosition();
			    		//manager.Publish();
			    		manager.x_err_right.p(0) = 1;
			    		action = 4;
			    		ROS_INFO("Good job guy, you've done!!!");
			        }
					break;

				default:
					action = 1;
			}

		manager.Publish();
		
		}
	 
	 }
		
	
	return 0;
}