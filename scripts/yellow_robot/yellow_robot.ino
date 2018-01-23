#include <ros.h>
#include <std_msgs/String.h>

char key;

// This is the handle for the node we will create.
ros::NodeHandle n_h;

// Function gets called every time there is a message
// published to the topic 
void message_callback(const std_msgs::String& key_press){
  key = key_press.data[0];
  if(key == 'U'){
    //digitalWrite(13,HIGH);
  }
}

// Initialize the subscriber
ros::Subscriber<std_msgs::String> sub1("keys", &message_callback);

// Declare the pins
int mot_l = 11;
int mot_r = 10;
int dir_1 = 6; 
int dir_2 = 7;
int dil_1 = 9; 
int dil_2 = 8;

int PWM_R = 70; //battery
int PWM_L = 75;
//int PWM_R = 50; // Power adapter
//int PWM_L = 55;

// Functions 

void set_dir(char dir){
  
  if (dir == 'F'){
    digitalWrite(13,HIGH);
    digitalWrite(dir_1,HIGH);
    digitalWrite(dir_2,LOW); 
    digitalWrite(dil_1,HIGH);
    digitalWrite(dil_2,LOW);     
  }
  if(dir == 'B'){ 
    digitalWrite(13,LOW);
    digitalWrite(dir_1,LOW);
    digitalWrite(dir_2,HIGH);
    digitalWrite(dil_1,LOW);
    digitalWrite(dil_2,HIGH); 
  }
  if(dir == 'L'){
    digitalWrite(dir_1,HIGH);
    digitalWrite(dir_2,LOW); 
    digitalWrite(dil_1,LOW);
    digitalWrite(dil_2,HIGH);
    }
  if(dir == 'R'){
    digitalWrite(dir_1,LOW);
    digitalWrite(dir_2,HIGH);
    digitalWrite(dil_1,HIGH);
    digitalWrite(dil_2,LOW);
    }
}

void go(){
  analogWrite(mot_l,PWM_L);
  analogWrite(mot_r,PWM_R);
}

void no_go(){
  analogWrite(mot_l,0);
  analogWrite(mot_r,0);
}


void setup(){

  pinMode(mot_l,OUTPUT);
  pinMode(mot_r,OUTPUT);
  pinMode(dir_1, OUTPUT);
  pinMode(dir_2, OUTPUT);


  // Initialize the node
  n_h.getHardware()->setBaud(9600);
  n_h.initNode();
  
  // Initialize
  n_h.subscribe(sub1);

}

void loop(){
  
  
  if(key == 'U'){
        //digitalWrite(mot_l,HIGH);
        //digitalWrite(13,HIGH);
	set_dir('F');
	go();
  }
  else if(key == 'D'){
        set_dir('B');
        //digitalWrite(mot_l,LOW);
        //digitalWrite(13,LOW);
	go();
  }
  else if(key == 'L'){
        set_dir('L');
	go();
  }
  else if(key == 'R'){
        set_dir('R');
	go();
  }
  else{
	no_go();
  }  
 
  // spinOnce is required for ROS communication.
  n_h.spinOnce();
  delay(1);
}





