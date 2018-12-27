#include <std_msgs/Int32MultiArray.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "sensor_msgs/Joy.h"
#include "math.h"
#include <time.h>

using namespace std;

///////////////////////////////////////////
float forwardMove = 3;
float successRadius = 0.22;
float sideMove = 2;
float alignAdjustW = 0.25;
float alignAdjustH = 0.25;
float Kp = 0.0018;
bool chooseSide = 0;		// 0 will make it move right for the first time it reaches controller 1
int nGatesFirstSection = 2; 


float nGates = 13;
float waypoints[13][4] = {
        {0,-2,0,0},         // Gate 0
        {1,0,0, 3.14 / 2}, // Gate 1
        {0,-2,0,0},
        {0,2,0,0},
        {1.5,0,0, 3.14 / 2 },  // Transition Section 1 / Section 2
        {2,0,0, 3.14 / 2},
        {0,2,0,0},
        {0,2,0,0},
        {0,2,0,0},
        {0,2,0,0},
        {0,2,0,0},
        {0,2,0,0},
        {0,2,0,0}

};


//////////////////////////////////////////
// Globals
mavros_msgs::State current_state;
geometry_msgs::PoseStamped target_pose;
//setup arming code bool
mavros_msgs::CommandBool arm_status;
mavros_msgs::SetMode offb_set_mode;
//xbox joystick control
bool xbox_control = true;
float xbox_pose[6] = {0,0,0,0,0,0};
bool xbox_flag = false;
tf::Quaternion quad_orientation;
float yaw = 0;
//Flight mode switch
int FlightMode = 1;

///////////////////////////////////////////////

int CP_W = 0;
int CP_H = 0;
int imgH = 0; 
int imgW = 0;
float bias[3] = {0,0,0};
float currentPose[3] = {0,0,0};
float bound = 0.58;
int hyp = 2.5;
bool moveFlag = 0;
bool alignFlag = 0;
int controllerMode = 0;
int currentGate = 0;
////////////////////////////////////////////////
void centerPointCallback(const std_msgs::Int32MultiArray& msg)
{
imgH = msg.data[0];
imgW = msg.data[1];
CP_W = msg.data[2]; 
CP_H = msg.data[3];
}

void init(){
	// Create and set target position
	target_pose.pose.position.x = 0;
	target_pose.pose.position.y = 0;
	target_pose.pose.position.z = 0.5;
	target_pose.pose.orientation.x = 0;
	target_pose.pose.orientation.y = 0;
	target_pose.pose.orientation.z = 0;
	target_pose.pose.orientation.w = 1;
	controllerMode = 1;
	currentGate = 0;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void xbox_cb(const sensor_msgs::Joy& msg);



void body_to_world(float body_x, float body_y, float body_z, float& global_x, float& global_y, float& global_z){
	global_x =  body_x*cos(yaw) - body_y*sin(yaw);
	global_y =  body_y*cos(yaw) + body_x*sin(yaw);
	global_z =  body_z;
	}


int main(int argc, char **argv)
{
	init();
  ros::init(argc, argv, "theMan_node");
  ros::NodeHandle nh;

	//Subscribers
	ros::Subscriber state_subscriber = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber pose_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
	ros::Subscriber joystick = nh.subscribe("/joy", 1000, xbox_cb);
	ros::Subscriber sub = nh.subscribe("/centerPoint", 1, centerPointCallback);

	// Publishers
	ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	//Service CLient
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	// Publish Rate
	ros::Rate rate(400.0);

	// wait for PixRacer
	//while(ros::ok() && current_state.connected){
	//	ros::spinOnce();
	//	rate.sleep();
	//}


	while(ros::ok()){

		if(xbox_flag){
			set_mode_client.call(offb_set_mode);
			arming_client.call(arm_status);
			xbox_flag = false;
		}

		// Publish the desired position
		pose_publisher.publish(target_pose);
				
		//xbox_control
		if(FlightMode == 1){
			//ROS_INFO("Flight Mode : 1");
                        float global_x = 0; float global_y = 0; float global_z = 0;
                        body_to_world(0.005*xbox_pose[0], 0.005*xbox_pose[1], 0.002*xbox_pose[2], global_x, global_y, global_z);

                        target_pose.pose.position.x = target_pose.pose.position.x + global_x;           
                        target_pose.pose.position.y = target_pose.pose.position.y + global_y;
                        target_pose.pose.position.z = target_pose.pose.position.z + global_z;
/*
			target_pose.pose.position.x += 0.005*xbox_pose[0] ;
			target_pose.pose.position.y += 0.005*xbox_pose[1] ;
			target_pose.pose.position.z += 0.002*xbox_pose[2] ;
*/
			yaw += 0.001 * xbox_pose[3];
			//target_pose.pose.orientation = tf::createQuaternionFromYaw(yaw);
			quad_orientation.setRPY(0,0,yaw);
			target_pose.pose.orientation.x = quad_orientation.x();
			target_pose.pose.orientation.y = quad_orientation.y();
			target_pose.pose.orientation.z = quad_orientation.z();
			target_pose.pose.orientation.w = quad_orientation.w();
		}
		if(FlightMode == 2){
		//ROS_INFO("Flight Mode : 2");
		float d = pow(target_pose.pose.position.x - currentPose[0], 2) + pow(target_pose.pose.position.y - currentPose[1], 2) + pow(target_pose.pose.position.z - currentPose[2], 2);
		
		if (sqrt(d) < successRadius && controllerMode == 1)
		{
			
			float global_x = 0; float global_y = 0; float global_z = 0;
			body_to_world(waypoints[currentGate][0], waypoints[currentGate][1], waypoints[currentGate][2], global_x, global_y, global_z);

			target_pose.pose.position.x = target_pose.pose.position.x + global_x;		
			target_pose.pose.position.y = target_pose.pose.position.y + global_y;
			target_pose.pose.position.z = target_pose.pose.position.z + global_z;

			quad_orientation.setRPY(0,0, waypoints[currentGate][3]);
                        target_pose.pose.orientation.x = quad_orientation.x();
                        target_pose.pose.orientation.y = quad_orientation.y();
                        target_pose.pose.orientation.z = quad_orientation.z();
                        target_pose.pose.orientation.w = quad_orientation.w();
		controllerMode = 2;
		ROS_INFO("Gate :  %i ", currentGate);
		ROS_INFO("Controller 1");

		}

		else if (sqrt(d) < successRadius && controllerMode == 2)
 		{
			//ROS_INFO("imgH %i", imgH);
			//ROS_INFO("imgW %i", imgW);
			//ROS_INFO("CP_W %i", CP_W);
			//ROS_INFO("CP_H %i", CP_H);
		
		
		if(CP_W < (1-bound)*imgW || CP_W > (bound)*imgW || CP_H < (1-bound)*imgH || CP_H > (bound)*imgH)
		{
			alignAdjustW = Kp*(CP_W - imgW / 2);

			alignAdjustH = Kp*(CP_H - imgH / 2);
                        

                        float global_x = 0; float global_y = 0; float global_z = 0;
                        body_to_world(0, alignAdjustW, alignAdjustH, global_x, global_y, global_z);

                        target_pose.pose.position.x = target_pose.pose.position.x - global_x;           
                        target_pose.pose.position.y = target_pose.pose.position.y - global_y;
                        target_pose.pose.position.z = target_pose.pose.position.z - global_z;

			controllerMode = 2;
		}
		else
		{
		controllerMode = 3;
		}
		ROS_INFO("Aligning : Controller Mode %i", controllerMode);
		}

                else if (sqrt(d) < successRadius && controllerMode == 3)
		{
                        float global_x = 0; float global_y = 0; float global_z = 0;
                        body_to_world(forwardMove, 0, 0, global_x, global_y, global_z);

                        target_pose.pose.position.x = target_pose.pose.position.x + global_x;           
                        target_pose.pose.position.y = target_pose.pose.position.y + global_y;
                        target_pose.pose.position.z = target_pose.pose.position.z + global_z;

			currentGate += 1;
                        controllerMode = 1;
			
			if(currentGate == nGates)
			FlightMode = 1;

		ROS_INFO("Going Forward : ");
		}
		}		
		//ROS_INFO("Desired y : %f", target_pose.pose.position.y);
		//ROS_INFO("Desired z : %f", target_pose.pose.position.z);
		//ROS_INFO("Current y : %f", pose[1]);
		//ROS_INFO("Current z : %f", pose[2]);
		//ROS_INFO("alignFlag : %i", alignFlag);
		
			
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;

}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	currentPose[0] = msg->pose.position.x;
	currentPose[1] = msg->pose.position.y;
	currentPose[2] = msg->pose.position.z;
}

//xbox callback
void xbox_cb(const sensor_msgs::Joy& msg){	
	//

	if(msg.buttons[2] == 1){
	//ARM = X
	xbox_flag = true;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	arm_status.request.value = true;
	}
	if(msg.buttons[3] == 1){
	//DISARM = B
	//bias[0] = pose[0];
        //bias[1] = pose[1];
        //bias[2] = pose[2];

//	target_pose.pose.position.x = bias[0];
//	target_pose.pose.position.y = bias[1];
//	target_pose.pose.position.z = bias[2];

//	ROS_INFO("Bias %f %f %f", bias[0],bias[1],bias[2]);

	xbox_flag = true;
	offb_set_mode.request.custom_mode = "STABILIZED";
	arm_status.request.value = false;
	}
	if(msg.buttons[0] == 1){
		FlightMode = 1;
		ROS_INFO("Joystick Control");
	}
	if(msg.buttons[1] == 1){
		FlightMode = 2;
		alignFlag = 0;
		ROS_INFO("Vision Control");
	}

	if(abs(msg.axes[3])>0.2) //X
		xbox_pose[0] = msg.axes[3];
	else
		xbox_pose[0] = 0;
	if(abs(msg.axes[2])>0.2) //Y
		xbox_pose[1] = msg.axes[2];
	else
		xbox_pose[1] = 0;
	if(abs(msg.axes[1])>0.2)  //Z
		xbox_pose[2] = msg.axes[1];
	else
		xbox_pose[2] = 0;
	if(abs(msg.axes[0])>0.2)  //YAW
		xbox_pose[3] = msg.axes[0];
	else
		xbox_pose[3] = 0;
}

