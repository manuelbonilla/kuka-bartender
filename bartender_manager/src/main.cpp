#include "BartenderManager.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bartender_manager");

	BartenderManager manager;

	std::vector<int> closure_value; 
	closure_value.resize(2);
	
	closure_value[0] = 1;
	closure_value[1] = 1;

	std::string s_l, s_r;
	s_l = "left";
	s_r = "right";

	manager.Init();

	//manager.DrinkSelection();

	while (ros::ok())
	{
		int action = 1;
		manager.DrinkSelection();

		while (action != 4)
		{	
			manager.n_.param<double>("threshold", manager.threshold, 0.15);

			manager.n_.param<float>("roll", manager.roll_b, 0);
		    manager.n_.param<float>("pitch", manager.pitch_b, -90);
		    manager.n_.param<float>("yaw", manager.yaw_b, 0);
		    
		    manager.Init();

			switch (action)
			{

				case(1):
					ROS_INFO("FIRST action");
					while (Equal(manager.x_err_right, manager.x_err_compare, manager.threshold) && Equal(manager.x_err_left, manager.x_err_compare, manager.threshold) && action == 1)
	        		{	
	        			manager.Grasping(closure_value, s_l);
	        			manager.Grasping(closure_value, s_r);

	        			ros::Duration(5).sleep();

			    		manager.x_err_right.p(0) = 1;
			    		action = 2;
			    		ROS_INFO("Both arm on targets, no i'm going to grasping and pouring in the glass");
		

	        			manager.ToGlass();			
			        }
					break;

				case(2):
					ROS_INFO("THIRD action");
					while (Equal(manager.x_err_right, manager.x_err_compare, manager.threshold) && Equal(manager.x_err_left, manager.x_err_compare, manager.threshold) && action == 2)
	        		{	
			        	ROS_INFO("Now it's time to pouring!!! ");

			    		closure_value[0] = 0;
						closure_value[1] = 0;
			    		manager.Grasping(closure_value, s_l);
	        			manager.Grasping(closure_value, s_r);

	        			ros::Duration(5).sleep();

	        			manager.InitialPosition();
			    		manager.x_err_right.p(0) = 1;
			    		action = 3;
			        }
					break;

				case(3):
					ROS_INFO("FOURTH action");
					while (Equal(manager.x_err_right, manager.x_err_compare, manager.threshold) && Equal(manager.x_err_left, manager.x_err_compare, manager.threshold) && action == 3)
	        		{	
			    		manager.x_err_right.p(0) = 1;
			    		action = 4;
			    		ROS_INFO("Good job guy, you've done!!!");

	        			ros::Duration(1).sleep();
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