// Capstone Project - Astro Motor Control

// TODO: 
// publish odometry. 
// verify timing of everything
// change rpm_control to servo style control
// can we publish once every subscriber call? Issue with latched messages...


#include <ros.h>

// The cmd_vel topic is a Twist message so you need to import that object 
// Twist msg is two Vector3 objects: LINEAR x y z, ANGULAR roll pitch yaw
#include <geometry_msgs/Twist.h>


// The encoder counts will be published as a two element array. For now, one element. 
#include <std_msgs/Int16.h>


// Required for ros 
ros::NodeHandle nh;


// The cmd_vel Twist message will be unpacked in to these variables
double cmd_vel_x;
double cmd_vel_yaw;
int r_ticks;
int l_ticks;


// This function gets executed each time a new message is published
// to the cmd_vel topic.   
void messageCb(const geometry_msgs::Twist& cmd_vel){

  
  // Unpack the variables 
  cmd_vel_x = cmd_vel.linear.x;
  cmd_vel_yaw = cmd_vel.angular.z;
  
  if(cmd_vel_x==0.5){
    // Toggle the LED if robot's going forward. 
    digitalWrite(13, HIGH-digitalRead(13));  // blink the led
  }

  // For debugging purposes
  if(cmd_vel_x==0.5) nh.loginfo("UP");
  if(cmd_vel_x==-0.5) nh.loginfo("DOWN");  
  if(cmd_vel_yaw==1.0 || cmd_vel_yaw==-1.0) nh.loginfo("yawing");
  
  
}

std_msgs::Int16 counts;

// necessary ros objects. Need one for each topic you want to subscribe/publish to
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("cmd_vel", &messageCb );
ros::Publisher pub_ticks("ticks", &counts);



void setup()
{
  pinMode(13, OUTPUT);
  
  nh.initNode();

  // Attach the subscriber object to the node handle
  nh.subscribe(cmd_vel_subscriber);
  nh.advertise(pub_ticks);
}

void loop()
{
  static int i = 0;
  // spinOnce is required for some reason.
  counts.data = i++;
  pub_ticks.publish(&counts);
  nh.spinOnce();
  
}
