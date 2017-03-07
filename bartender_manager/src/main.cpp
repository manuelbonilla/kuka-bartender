#include "BartenderManager.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bartender_manager");

	BartenderManager manager;

	std::vector<int> closure_value; 
	std::vector<int> opening_value; 

	std::string s_l, s_r;

	closure_value.resize(2);
	opening_value.resize(2);

	closure_value[0] = 1;
	closure_value[1] = 1;

	opening_value[0] = 0;
	opening_value[1] = 0;

	s_l = "left";
	s_r = "right";

	manager.Init();

	int action = 1;

	bool finished = false;
	bool fin_l = false;
	bool fin_r = false;
	
	while (ros::ok())
	{
		
		manager.DrinkSelection();

		//**********************************************************************************************************************//
        //  							Sequence of actions aimed at the realization of cocktail							    //
        //**********************************************************************************************************************//

		//**********************************FIRST ACTION: To the bottles**********************************************************

		while (action == 1)
		{
			manager.Publish();

			if ( Equal(manager.x_err_right, manager.x_err_compare, manager.threshold) && !fin_r)
			{	
				ROS_INFO("DESTRO arrivato!!");
				manager.Grasping(closure_value, s_r);
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}

			if ( Equal(manager.x_err_left, manager.x_err_compare, manager.threshold) && !fin_l)
			{	
				ROS_INFO("SINISTRO arrivato!!");
				manager.Grasping(closure_value, s_l);	
				fin_l = !fin_l;
				manager.msg_left.arrived = true;
			}

			if ( fin_r && fin_l )
				{
					ROS_INFO("FINITO!!");
					action = 2;
					fin_l = false;
					fin_r = false;
				}

		}

		//**********************************SECOND ACTION: To the glass**********************************************************

		manager.ToGlass();

		while (action == 2)
		{
			manager.Publish();

			if ( Equal(manager.x_err_right, manager.x_err_compare, manager.threshold) && !fin_r)
			{	
				ROS_INFO("DESTRO arrivato!!");
				fin_r = !fin_r;	
				manager.msg_right.arrived = true;
			}

			if ( Equal(manager.x_err_left, manager.x_err_compare, manager.threshold) && !fin_l)
			{	
				ROS_INFO("SINISTRO arrivato!!");
				fin_l = !fin_l;
				manager.msg_left.arrived = true;
			}

			if ( fin_r && fin_l )
			{
				ROS_INFO("FINITO!!");
				action = 3;
			}

		}
	}
		
	
	return 0;
}