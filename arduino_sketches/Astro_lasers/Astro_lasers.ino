#include <Wire.h>
#include <VL53L0X.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>



#define led 12
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

ros::NodeHandle nh;

std_msgs::String status;

ros::Publisher Status("Status", &status);


void setup()
{

  //nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(Status);
  Serial.begin(57600);
  Wire.begin();

  sensor1.init();
  sensor1.setTimeout(500);
  
  
  //initialize the digital pins for all the interupt pins of the sensor
  pinMode(1, OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5,OUTPUT);

 
  //Initialize all the pins to low   
  digitalWrite(1,LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  
  
  delay(50);



  pinMode(1,INPUT);
  sensor1.init(true);
  delay(50);
  sensor1.setAddress((uint8_t)24);
  
  pinMode(2,INPUT);
  sensor2.init(true);
  delay(50);
  sensor2.setAddress((uint8_t)25);

  pinMode(3,INPUT);
  sensor3.init(true);
  delay(50);
  sensor3.setAddress((uint8_t)26);
  
  pinMode(4,INPUT);
  sensor4.init(true);
  delay(50);
  sensor4.setAddress((uint8_t)27);
  
  pinMode(5,INPUT);
  sensor5.init(true);
  delay(50);
  sensor5.setAddress((uint8_t)28);


  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  sensor4.startContinuous();
  sensor5.startContinuous();

  
}

void loop()
{

  if (sensor2.readRangeContinuousMillimeters()<300)
  {
    status.data = "1";
    Status.publish (&status);
  }
  
  else if (sensor4.readRangeContinuousMillimeters() <300)
  {
    status.data = "2";
    Status.publish (&status);
  }

  else if (sensor5.readRangeContinuousMillimeters() <300)
  {
    status.data = "3";
    Status.publish (&status);
  }
  
  else 
  {
    status.data = "0";
    Status.publish (&status);
  }

  //delay(100);

  Serial.print ("Distance 1 = ");
  Serial.println (sensor2.readRangeContinuousMillimeters());  
 
  //delay(100);
  
  Serial.print ("Distance 2 = ");
  Serial.println (sensor4.readRangeContinuousMillimeters());
  
  nh.spinOnce();
  //delay(100);
  // Serial.println(i2cdetect(sensor));
}
