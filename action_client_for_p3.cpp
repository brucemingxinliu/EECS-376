// example_navigator_action_client: 
// wsn, April, 2016
// illustrates use of navigator action server called "navigatorActionServer"

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <navigator/navigatorAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
bool g_alarm;

geometry_msgs::PoseStamped g_desired_pose;
int nav_pose;

void alarmCB(const std_msgs::Float64& message_holder)
{
    g_alarm = true;
    

}//lidar alarm void declare...
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void navigatorDoneCb(const actionlib::SimpleClientGoalState& state,
        const navigator::navigatorResultConstPtr& result) {

    nav_pose = result->return_code;

   /* ROS_INFO(" navigatorDoneCb: server responded with state [%s]", state.toString().c_str());
    g_navigator_rtn_code=result->return_code;

    ROS_INFO("got object code response = %d; ",g_navigator_rtn_code);
    if (g_navigator_rtn_code==navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED) {
        ROS_WARN("destination code not recognized");
    }
    else if (g_navigator_rtn_code==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
        ROS_INFO("reached desired location!");
    }
    else {
        ROS_WARN("desired pose not reached!");
    }*/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_navigator_action_client"); // name this node 
    ros::NodeHandle nh; //standard ros node handle    
    geometry_msgs::Quaternion quat;
    g_alarm = nh.subscribe("lidar_alarm",1,alarmCB);
    actionlib::SimpleActionClient<navigator::navigatorAction> navigator_ac("navigatorActionServer", true);
    
    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = navigator_ac.waitForServer(ros::Duration(0.5)); // 
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("connected to navigator action server"); // if here, then we connected to the server; 
     
    navigator::navigatorGoal navigation_goal;

    navigation_goal.location_code=navigator::navigatorGoal::HOME;
    geometry_msgs::Pose pose;
    pose.position.x = 1.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0;
        

    if(g_alarm = true){
    ROS_WARN("LIDAR ALARM IS SOUNDED");
   
    quat = convertPlanarPhi2Quaternion(0.0); // get a quaternion corresponding to this heading
    pose.orientation = quat;   
    pose.position.x = 8.0;
    pose.position.y = 0.0;
 navigation_goal.desired_pose.push_back(g_desired_pose);

    quat = convertPlanarPhi2Quaternion(0.3); // get a quaternion corresponding to this heading
    pose.orientation = quat;   
    pose.position.x = 3.0;
    pose.position.y = 3.0;
 navigation_goal.desired_pose.push_back(g_desired_pose);

    quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
    pose.orientation = quat;   
    pose.position.x = 2.0;
    pose.position.y = 3.0;
 navigation_goal.desired_pose.push_back(g_desired_pose);
   
    g_desired_pose.pose = pose;
}
   else{
    quat = convertPlanarPhi2Quaternion(0.0); // get a quaternion corresponding to this heading
    pose.orientation = quat;   
    pose.position.x = 8.0;
    pose.position.y = 2.0;
    navigation_goal.desired_pose.push_back(g_desired_pose);

    quat = convertPlanarPhi2Quaternion(0.3); // get a quaternion corresponding to this heading
    pose.orientation = quat;
    pose.position.x = 3.0;
    pose.position.y = 1.0;
    navigation_goal.desired_pose.push_back(g_desired_pose);

    quat = convertPlanarPhi2Quaternion(1.57); // get a quaternion corresponding to this heading
    pose.orientation = quat;  
    pose.position.x = 1.0;
    pose.position.y = 1.0;
    navigation_goal.desired_pose.push_back(g_desired_pose);
   
    g_desired_pose.pose = pose;
}
   
    
    ROS_INFO("sending goal: ");
        navigator_ac.sendGoal(navigation_goal,&navigatorDoneCb); // we could also name additional callback functions here, if desired

        
        bool finished_before_timeout = navigator_ac.waitForResult(ros::Duration(30.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result ");
            return 1;
        }
    

    return 0;
}
