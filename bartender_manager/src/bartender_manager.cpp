#include "BartenderManager.h"

using namespace std;

BartenderManager::BartenderManager() 
{
    //sub_bartender_check_ = n_.subscribe("/bartender/cmd_check", 1, &BartenderManager::checkCallback, this);

    //pub_bartender_cmd_ = n_.advertise<bartender_control::bartender_msg>("/bartender/des_position", 10);
    //anymal_bartender_srv_ = n_.advertiseService("/bartender/barteder_manager_srv", &BartenderManager::parseSrvAction, this);
    pub_bartender_cmd_ = n_.advertise<bartender_control::bartender_msg>("/bartender/des_position", 10);

}
BartenderManager::~BartenderManager() {}

void BartenderManager::EulerToQuaternion(float R, float P, float Y)
{
	double q[4];
	double t0 = std::cos((3.142 * Y/ 180)*0.5f);
	double t1 = std::sin((3.142 * Y/ 180)*0.5f);
	double t2 = std::cos((3.142 * R/ 180)*0.5f);
	double t3 = std::sin((3.142 * R/ 180)*0.5f);
	double t4 = std::cos((3.142 * P/ 180)*0.5f);
	double t5 = std::sin((3.142 * P/ 180)*0.5f);

	q[0] = t0 * t2 * t4 + t1 * t3 * t5;
	q[1] = t0 * t3 * t4 - t1 * t2 * t5;
	q[2] = t0 * t2 * t5 + t1 * t3 * t4;
	q[3] = t1 * t2 * t4 - t0 * t3 * t5;

}

void BartenderManager::InsertCartPosEE ()
{
	
	bartender_control::bartender_msg msg;
	
	//insert desired EE orientation in degrees
	float des_Roll_EE, des_Pitch_EE, des_Yaw_EE;

	cout << "Please enter RPY desired orientation of End-Effector (in Degrees)" << endl;
	cout << "Roll: ";
	cin >> des_Roll_EE;
	cout << endl  << "Pitch: ";
	cin >> des_Pitch_EE;
	cout << endl << "Yaw: ";
	cin >> des_Yaw_EE;
	cout << endl << "Your desired RPY orientation is: " << des_Roll_EE << "," << des_Pitch_EE << "," << des_Yaw_EE << endl;
	
	//From RPY to quaternion	
	double q[4];
	double t0 = std::cos((3.142 * des_Yaw_EE/ 180)*0.5f);
	double t1 = std::sin((3.142 * des_Yaw_EE/ 180)*0.5f);
	double t2 = std::cos((3.142 * des_Roll_EE/ 180)*0.5f);
	double t3 = std::sin((3.142 * des_Roll_EE/ 180)*0.5f);
	double t4 = std::cos((3.142 * des_Pitch_EE/ 180)*0.5f);
	double t5 = std::sin((3.142 * des_Pitch_EE/ 180)*0.5f);

	q[0] = t0 * t2 * t4 + t1 * t3 * t5;
	q[1] = t0 * t3 * t4 - t1 * t2 * t5;
	q[2] = t0 * t2 * t5 + t1 * t3 * t4;
	q[3] = t1 * t2 * t4 - t0 * t3 * t5;
	
	//insert desired EE position
	cout << endl << "Please enter Cartesian desired position of End-Effector" << endl;
	cout << "X: ";
	cin >> x_des_.p(0);
	cout << endl  << "Y: ";
	cin >> x_des_.p(1);
	cout << endl << "Z: ";
	cin >> x_des_.p(2);
	cout << endl << "Your desired cartesian position is: " << x_des_.p(0) << "," << x_des_.p(1) << "," << x_des_.p(2) << endl;

	//Intitialization of x_des_ with EE desired poition and a fized orientation (0,0,0)
	x_des_ = KDL::Frame(KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]), x_des_.p);

	//Put desired EE position in the control message
    msg.des_frame.position.x = x_des_.p(0);
    msg.des_frame.position.y = x_des_.p(1);
    msg.des_frame.position.z = x_des_.p(2);

    //Put desired EE orientation in the control message
    msg.des_frame.orientation.x = q[0];
    msg.des_frame.orientation.y = q[1];
    msg.des_frame.orientation.z = q[2];
    msg.des_frame.orientation.w = q[3];
	
	//debug
	cout << endl << msg.des_frame.position << endl;
	cout << endl << msg.des_frame.orientation << endl;
	pub_bartender_cmd_.publish(msg);
	//ros::spinOnce();


}

std::vector<float> BartenderManager::doFollow(float x, float y, float z, bool init_trajectory)
{
    //std::vector<float> final_position(2);
    //call_count_trunk_ = 0;
    KDL::Frame x_des;


    x_des = KDL::Frame(KDL::Rotation::Quaternion(0, 0, 0, 1), KDL::Vector(x, y, z));
    /*while(call_count_trunk_ == 0){
        ros::spinOnce();
    }

    call_count_trunk_ = 0;

    if(init_trajectory){
        trunk_init_ = x_;
        init_trajectory = false;
    }*/
    
    /*x_des = trunk_init_ * x_des;


    std::cout <<"x_des" << x_des.p << std::endl;
    std::cout << "x_" << x_.p << std::endl;


    KDL::Frame trunk_des;
    double relative_angle_rad;
    int relative_angle_deg;
    std::string action;
    double distance;


    while(!Equal(x_des.p,x_.p,0.05)){
        trunk_des.p = x_des.p - x_.p;	//error between desired position and actual position
        trunk_des.p = x_.M.Inverse() * trunk_des.p;	//Gives back inverse transformation of frame x_

        distance = sqrt(pow(trunk_des.p(0),2) + pow(trunk_des.p(1),2));

        relative_angle_rad = atan2(trunk_des.p(1), trunk_des.p(0));
        relative_angle_deg = (int)(relative_angle_rad * 180/3.14); 


        if(abs(relative_angle_deg) >= 2){
            doAction(9,"walk_adj", relative_angle_deg);
        }else if(distance > 0.05){
            if(trunk_des.p(0) > 0){
                action = "walk_fw";
            }else if(trunk_des.p(0) < 0){
                action = "walk_bw";
            }else if(trunk_des.p(1) > 0){
                action = "walk_lx";
            }else{
                action = "walk_rx";
            }
            doAction(8, action, 1);
        }

        trunk_semaphore_ = true;
        while(call_count_trunk_ == 0){
            ros::spinOnce();
        }

        call_count_trunk_ = 0;
        std::cout <<"x_des" << x_des.p << std::endl;
        std::cout << "x_" << x_.p << std::endl;

    }

    x_ = trunk_init_.Inverse() * x_;
    std::cout << "final_position: " << x_.p(0) << " " << x_.p(1) << std::endl;

    final_position[0] = x_.p(0);
    final_position[1] = x_.p(1);*/

    //return final_position;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "bartender_manager");

	BartenderManager manager;

	manager.InsertCartPosEE();

	
	//manager.frameFromAngle(180);

	//manager.doFollow(x_des_.p(0), x_des_.p(1), x_des_.p(2), true);

	
	ros::spinOnce();
	
	return 0;
}
