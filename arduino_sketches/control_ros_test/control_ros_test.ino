// -------- INCLUDE STATEMENTS --------
#include <ros.h>
#include <std_msgs/String.h>
// The cmd_vel topic is a Twist message so you need to import that object 
// Twist msg is two Vector3 objects: LINEAR x y z, ANGULAR roll pitch yaw
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// -------- DEFINE STATEMENTS AND GLOBAL VARIABLES --------
#define BIT(a) (1<<(a)) // turns on the specified bit within that register

// Right motor
#define R_MOT (10)
#define R_D1 (8)
#define R_D2 (6)

// Left motor
#define L_MOT (9)
#define L_D1 (4)
#define L_D2 (5)

// Encoder info
#define TICKS_REV_R (497)
#define TICKS_REV_L (497)
#define L_ENC (2) // pin INT0
#define L_ENC1 (11)
#define R_ENC (3) // pin INT1
#define R_ENC2 (12)

// CONTROL SYSTEM VARIABLES
// In present configuration, 16ms per compare match interrupt generated
// The frequency and period of one control loop iteration and is determined
// both by clock prescalar and how many overflows are required to trigger cs_start. 

// Timing:

volatile bool cs_start = false;
volatile int waste = 0;
#define MS_PER_CMP (16)
// number of compare mathces before cs_start is triggered
#define N_CMP (6)
int cmp_count = 0;

// Period of one control loop iteration = dt

double dt = (MS_PER_CMP*N_CMP*0.001);

double v_lin;
double input_r_rpm;
double input_l_rpm;
double ratio;
int e_r,u_r_rpm,u_r_pwm ,e_l,u_l_rpm,u_l_pwm; //error and control signals
double kp=20; //proportional term gain

// Feedback variables

double r_rpm; 
double l_rpm; 
volatile int r_ticks = 0;
volatile int l_ticks = 0;

// Testing variables

int delay_s;
int target;
int iter;
int targit;

int PWM;


// ROS Communication

ros::NodeHandle nh;

// The cmd_vel Twist message will be unpacked in to these variables
double cmd_vel_x;
double cmd_vel_yaw;

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



// necessary ros object. Need one for each topic you want to subscribe to
ros::Subscriber<geometry_msgs::Twist> cmd_vel_subscriber("cmd_vel", &messageCb );

std_msgs::Int16 r_counts;
std_msgs::Int16 l_counts;

ros::Publisher pub_ticks_r("r_ticks", &r_counts);
ros::Publisher pub_ticks_l("l_ticks", &l_counts);

void setup() {

  // ROS communication initialization
  nh.initNode();

  // Attach the subscriber object to the node handle
  nh.subscribe(cmd_vel_subscriber);  
  nh.advertise(pub_ticks_r);
  nh.advertise(pub_ticks_l);
  
  // Configure pins.   
  pinMode(13,OUTPUT);
  pinMode(R_MOT,OUTPUT);
  pinMode(R_D1,OUTPUT);
  pinMode(R_D2,OUTPUT);
  pinMode(R_ENC,INPUT_PULLUP);   
  pinMode(L_MOT,OUTPUT);
  pinMode(L_D1,OUTPUT);
  pinMode(L_D2,OUTPUT);
  pinMode(L_ENC,INPUT_PULLUP);
  set_direction(0);

  // SET TIMER REGISTERS 
  
  // With 1024 prescalar, compare match at 250ticks = 16ms. 
 
  TCCR2A = 0;
  TCCR2B = 0;  
  TCCR2B = BIT(CS22) | BIT(CS21) | BIT(CS20); 
  TIMSK2 = BIT(OCIE2A); //enable interrupt
  OCR2A = 250; 
  
  
  // Set external interrupt registers
  // enable interrupts on puns 2 and 3
  
  EIMSK = 0;
  EIMSK = BIT(INT0) | BIT(INT1); 
  EICRA = BIT(ISC11) | BIT(ISC10) | BIT(ISC01)| BIT(ISC00); 
 
  // To do: Incorporate Serial Communication  
  
  //Serial.begin(57600);

}

void loop() {

  while(!cs_start){waste++;} 
  

// CONTROL LOOP BEGINS HERE (EVERY dt = n_cmp*ms_per_cmp ms)            
  
  
// 1. RECEIVE COMMAND (INPUT)
// 155 is the conversion between m/s and RPM for 0.06191m radius wheels

input_r_rpm = cmd_vel_x * 155;
input_l_rpm = cmd_vel_x * 155;

  

  
// 2. READ SENSOR (OUTPUT)

  r_rpm = 60*(r_ticks/(double)TICKS_REV_R) / dt;
  l_rpm = 60*(l_ticks/(double)TICKS_REV_L) / dt;
  r_counts.data = r_rpm*100;
  l_counts.data = r_ticks;
  
  r_ticks = 0; //reset tick counts  
  l_ticks = 0;


  
//  3. COMPUTE ERROR SIGNAL (E = INPUT - OUTPUT)

  e_r = (input_r_rpm - r_rpm);
  e_l = (input_l_rpm - l_rpm);
  
  
  
//  4. COMPUTE CONTROL SIGNAL (U)
//  
//  todo: determine the controller, write data to files. 
//  current controller = proportional control (kp);
//  If error is negative, reverse motor direction
//  using set_x_mot() 1 = forward, 0 = back 
  
  if(e_r < 0 ) set_r_mot(0); 
  else set_r_mot(1);

  if(e_l < 0) set_l_mot(0); 
  else set_l_mot(1);

  u_r_rpm = kp*abs(e_r);
  u_l_rpm = kp*abs(e_l);

  // Convert from rpm to PWM
  // Assume a maximum allowed 100 rpm to map to 255 PWM

  // Saturate Inputs 

  if(u_r_rpm > 255) u_r_pwm = 255; 
  if(u_l_rpm > 255) u_l_pwm = 255; 
  
  
  
// 5. WRITE ACTUATORS WITH CONTROL SIGNAL
             
  analogWrite(L_MOT,u_l_rpm);
  analogWrite(R_MOT,u_r_rpm);
  //analogWrite(L_MOT,u_l_rpm);


  
// 6. MISCELLANEOUS 
  
  //Serial.print(u_r);
  //Serial.print("  ");
  //Serial.println(r_rpm);
  //Serial.print("error: ");
  //Serial.print(u_r_rpm);
  //Serial.print("   r_rpm: ");
  //Serial.print(r_rpm);
  //Serial.println();
  cs_start = false; 

// 7. ROS SPIN
  pub_ticks_r.publish(&r_counts);
  pub_ticks_l.publish(&l_counts);
  nh.spinOnce();

}


// INTERRUPT ROUTINES

ISR(INT0_vect){
  l_ticks++;
}

ISR(INT1_vect){
  r_ticks++;
}



ISR(TIMER2_COMPA_vect){
  cmp_count++;
  if(cmp_count==N_CMP){
    cs_start = true;
    cmp_count = 0;
  }
}


// MOTOR DIRECTION FUNCTIONS

void set_r_mot(int dir){
  
  if(dir==1){
    
    // go forward
    digitalWrite(R_D1,LOW);
    digitalWrite(R_D2,HIGH);
    
  }else{
    
    // go backward
    digitalWrite(R_D1,HIGH);
    digitalWrite(R_D2,LOW);
    
  }
}

void set_l_mot(int dir){

  if(dir==1){
    digitalWrite(L_D1,HIGH);
    digitalWrite(L_D2,LOW);
  }else{
    digitalWrite(L_D1,LOW);
    digitalWrite(L_D2,HIGH);
  }
  
}
void set_direction(int dir){

  // direction is what direction you want, d1 and d2 
  
  if(dir==0){                 //go straight
    digitalWrite(R_D1,LOW);
    digitalWrite(R_D2,HIGH);
    digitalWrite(L_D1,HIGH);
    digitalWrite(L_D2,LOW);
  }
  else if(dir==1){            // turn right 
    digitalWrite(R_D1,LOW);
    digitalWrite(R_D2,HIGH);
    digitalWrite(L_D1,LOW);
    digitalWrite(L_D2,HIGH);
  }
   else if(dir==2){
    digitalWrite(R_D1,HIGH); //turn left 
    digitalWrite(R_D2,LOW);
    digitalWrite(L_D1,HIGH);
    digitalWrite(L_D2,LOW);
   }
   else if(dir==3){
    digitalWrite(R_D1,HIGH);
    digitalWrite(R_D2,LOW);
    digitalWrite(L_D1,LOW);
    digitalWrite(L_D2,HIGH);
   }
}


