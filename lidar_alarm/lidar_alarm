#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 
#include <std_srvs/Trigger.h>

const double MIN_SAFE_DISTANCE = 1.0; // set alarm if anything is within 0.5m of the front of robot

// these values to be set within the laser callback

float ping_dist_in_front_=3.0;

int ping_index_=-1;
double angle_min_=-0.029;
double angle_max_= 0.029;
double angle_increment_=0.01;
double range_min_ = -500.0;
double range_max_ = 500.0;
bool laser_alarm_=false;
double ping_index_start;
double ping_index_end;

int ping_index_start_int;
int ping_index_end_int;
bool encounteredLowRange; 

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
ros::ServiceClient client;
ros::ServiceClient client_;
std_srvs::Trigger trig;
std_srvs::Trigger reset;

// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_index_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval


	
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_index_start = (-0.78 -angle_min_)/angle_increment_;
        ping_index_end = (0.78 -angle_min_)/angle_increment_;

	ping_index_start_int = (int)ping_index_start;
	ping_index_end_int = (int)ping_index_end;
        ROS_INFO("LIDAR setup: i = %d , r = %d",ping_index_start, ping_index_end);
        
    }
   encounteredLowRange=false; 
    
   for (int i = ping_index_start_int; i <= ping_index_end_int; i++){
   ping_dist_in_front_ = laser_scan.ranges[i];
   ROS_INFO("ping index start is = %d", ping_index_start_int);
   ROS_INFO("ping index end is  = %d" , ping_index_end_int);  
   ROS_INFO("ping dist in front = %f",ping_dist_in_front_);
   if (ping_dist_in_front_<MIN_SAFE_DISTANCE) {
       ROS_WARN("DANGER, WILL ROBINSON!!");
       laser_alarm_=true;
       client.call(trig);
       encounteredLowRange = true;
       
       
   }
   else if ((ping_dist_in_front_>MIN_SAFE_DISTANCE) & encounteredLowRange == false){
       laser_alarm_=false;
       client_.call(reset);
        }

       
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
   }
   }


int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
 
    client = nh.serviceClient<std_srvs::Trigger>("estop_service");
     client_ = nh.serviceClient<std_srvs::Trigger>("clear_estop_service");
   
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected to service");
    

    //ros::ServiceClient client__ = nh.serviceClient<std_srvs::Trigger>("flush_path_queue_service");
   // std_srvs::Trigger flush;
   // client__.call(flush);
    

    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1, laserCallback);
  //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive


   
     ros::spin(); 

       
    return 0; // should never get here, unless roscore dies
}
