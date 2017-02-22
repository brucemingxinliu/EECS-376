


//so far, the lidar alarm topic is subscribed. However no poses have been given in the client, and still needs to figure out how to send poses via action to the action server. 
#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include<navigator/navigatorAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

class Navigator {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation
    actionlib::SimpleActionServer<navigator::navigatorAction> navigator_as_;
    
    // here are some message types to communicate with our client(s)
    navigator::navigatorGoal goal_; // goal message, received from client
    navigator::navigatorResult result_; // put results here, to be sent back to the client when done w/ goal
    navigator::navigatorFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client

    int navigate_home();
    int navigate_to_table();
    int navigate_to_pose(geometry_msgs::PoseStamped goal_pose);
    
public:
    Navigator(); //define the body of the constructor outside of class definition

    ~Navigator(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal);
};



Navigator::Navigator() :
   navigator_as_(nh_, "navigatorActionServer", boost::bind(&Navigator::executeCB, this, _1),false) {
    ROS_INFO("in constructor of navigator...");
    // do any other desired initializations here...specific to your implementation

    navigator_as_.start(); //start the server running
}

//specialized function: DUMMY...JUST RETURN SUCCESS...fix this
//this SHOULD do the hard work of navigating to HOME
int Navigator::navigate_home() { 
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
} 
int Navigator::navigate_to_table() { 
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED; //just say we were successful
}
int Navigator::navigate_to_pose(geometry_msgs::PoseStamped goal_pose) {
    return navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
}
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 


// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}
double min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
} 

//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}



//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 
 dist = 0.0; //FALSE!!
 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    heading = 0.0; //FALSE!!
 }

}
void Navigator::executeCB(const actionlib::SimpleActionServer<navigator::navigatorAction>::GoalConstPtr& goal) {
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose desired_pose;
    geometry_msgs::PoseStamped pose_id;
    pose_id = goal->desired_pose;
    

   // int destination_id = goal->desired.pose;
   /* geometry_msgs::PoseStamped destination_pose;
    int navigation_status;
    int n_pts = destination_id.size();
    for(int i=1;n_pts;i++){



}
/*
    /*if (destination_id==navigator::navigatorGoal::COORDS) {
        destination_pose=goal->desired_pose;
    }
    
    switch(destination_id) {
        case navigator::navigatorGoal::HOME: 
              //specialized function to navigate to pre-defined HOME coords
               navigation_status = navigate_home(); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached home");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate home!");
                   navigator_as_.setAborted(result_);
               }
               break;
        case navigator::navigatorGoal::TABLE: 
              //specialized function to navigate to pre-defined TABLE coords
               navigation_status = navigate_to_table(); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached table");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to table!");
                   navigator_as_.setAborted(result_);
               }
               break;
        case navigator::navigatorGoal::COORDS: 
              //more general function to navigate to specified pose:
              destination_pose=goal->desired_pose;
               navigation_status = navigate_to_pose(destination_pose); 
               if (navigation_status==navigator::navigatorResult::DESIRED_POSE_ACHIEVED) {
                   ROS_INFO("reached desired pose");
                   result_.return_code = navigator::navigatorResult::DESIRED_POSE_ACHIEVED;
                   navigator_as_.setSucceeded(result_);
               }
               else {
                   ROS_WARN("could not navigate to desired pose!");
                   navigator_as_.setAborted(result_);
               }
               break;               
               
        default:
             ROS_WARN("this location ID is not implemented");
             result_.return_code = navigator::navigatorResult::DESTINATION_CODE_UNRECOGNIZED; 
             navigator_as_.setAborted(result_);
            }
  */
}


 //ros::NodeHandle n;
 //ros::Publisher g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1); 
//the topic is published to robot0...


int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_action_server"); // name this node 

    ROS_INFO("instantiating the navigation action server: ");
    ros::NodeHandle n;
    Navigator navigator_as; // create an instance of the class "ObjectFinder"

    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    while (ros::ok()) {
        ros::spinOnce(); //normally, can simply do: ros::spin();  
        // for debug, induce a halt if we ever get our client/server communications out of sync
    }

    return 0;
}
