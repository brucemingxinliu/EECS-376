#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <new_action_server/newAction.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


using namespace std;

bool g_alarm;//declare a global variable of statement

void alarmCB(const std_msgs::Float64& message_holder)
{
   g_alarm = true;

}


geometry_msgs::PoseStamped g_desired_pose;

void doneCb(const actionlib::SimpleClientGoalState& state,const new_action_server::myResultConstPtr& result)
{

	result->x_posestamps;
	result->y_posestamps;
	result->z_posestamps
}

int main(int argc, char**argv)
{
   ros::init(argc, argv,"new_action_client");
   ros::NodeHandle n;
   g_alarm = n.subscribe("lidar_alarm",1,alarmCB);

   new_action_server::newGoal goal;
   actionlib::SimpleActionClient<new_action_server::newAction> action_client("new_action",true);
   ROS_INFO("Waiting for new_action_server");
   bool server_exisits = action_client.waitForServer(ros::Duration(5.0));
   if(!server_exisits)
   {
   	  ROS_WARN("Halting...");
   	  return 0;
   }
   ROS_INFO("Connected to server!");

    geometry_msgs::Quaternion quat;
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;

    new_action_server::new_action_serverGoal;
    double x;
    double y;
    double z;
    while(ros::ok())
    {
    	std::cout<<"Enter a x coordinate for the robot: ";
    	std::cin>>x;
    	std::cout<<"Enter a y coordinate for the robot: ";
    	std::cin>>y;
    	std::cout<<"Enter a yaw rate for the robot: ";
    	std::cin>>z;

    
    goal.x_posestamps = x;
    goal.y_posestamps = y;
    goal.z_posestamps = z;
  
    ROS_INFO("sending goal: ");
    action_client.sendGoal(goal,&doneCb);
    }
  /*  
    pose.position.x = 1.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;


    quat = convertPlanarPhi2Quaternion(0.0); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.x=8.0; // say desired y-coord is 1.0
    pose_stamped.pose.position.y=0.0;
    pose_stamped.pose = pose;
    newAction.request.nav_path.poses.push_back(pose_stamped);
    
    quat = convertPlanarPhi2Quaternion(0.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=3.0;
    pose_stamped.pose.position.y=3.0;
     pose_stamped.pose = pose;
     newAction.request.nav_path.poses.push_back(pose_stamped);


    quat = convertPlanarPhi2Quaternion(1.57);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=2.0;
    pose_stamped.pose.position.y=3.0;
    pose_stamped.pose = pose;
    newAction.request.nav_path.poses.push_back(pose_stamped);

    */
   
    if(g_alarm = true){
    ROS_WARN("LIDAR ALARM IS SOUNDED");
    goal.alarm = 1;
    action_client.sendGoal(goal);
    





}


}


