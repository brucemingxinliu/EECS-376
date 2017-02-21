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
int main(int argc, char**argv)
{
   ros::init(argc, argv,"new_action_client_node");
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

    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
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
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;
    
    quat = convertPlanarPhi2Quaternion(0.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=3.0;
    pose_stamped.pose.position.y=3.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.57);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=2.0;
    pose_stamped.pose.position.y=3.0;
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;

    
    quat = convertPlanarPhi2Quaternion(-0.03);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=10.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;

    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=25.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(0.001);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=35.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;

    quat = convertPlanarPhi2Quaternion(1.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=40.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;



    quat = convertPlanarPhi2Quaternion(1.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(3.1);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(3.1);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(3.1);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(3.1);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;

    
    quat = convertPlanarPhi2Quaternion(3.1);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;



    quat = convertPlanarPhi2Quaternion(2.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(2.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=70.0;
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(2.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;

    
    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    quat = convertPlanarPhi2Quaternion(1.5);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y=80.0;
     goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;


    if(g_alarm = true){
    ROS_WARN("LIDAR ALARM IS SOUNDED");
    
    quat = convertPlanarPhi2Quaternion(0.3);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.x=3.0;
    pose_stamped.pose.position.y=3.0;
    goal.pose_stamped.pose.orientation = pose_stamped.pose.orientation;
    goal.pose_stamped.pose.position.x =  pose_stamped.pose.position.x;
    goal.pose_stamped.pose.position.y = pose_stamped.pose.position.y;






}
}

