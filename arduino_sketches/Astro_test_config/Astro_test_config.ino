// Capstone Team 16
// Concordia 2017 - 2018

// Purpose of this program is to ensure that the arduino pins are configured correctly. 
// Expected motor behaviour: straight forward for 2s, turn right 2s, turn left 2s, go backwards 2s, then stop for 2s and repeat. 


// Right motor
#define R_MOT (10)
#define R_D1 (6)
#define R_D2 (8)

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

void setup() {

  // Configure pins.  

  pinMode(R_MOT,OUTPUT);
  pinMode(R_D1,OUTPUT);
  pinMode(R_D2,OUTPUT);
  pinMode(R_ENC,INPUT_PULLUP);  
  pinMode(R_ENC2,INPUT_PULLUP); 
  pinMode(L_MOT,OUTPUT);
  pinMode(L_D1,OUTPUT);
  pinMode(L_D2,OUTPUT);
  pinMode(L_ENC,INPUT_PULLUP);
  pinMode(L_ENC1,INPUT_PULLUP);
}

void loop() {

	// Go forward
	set_direction(0);
	analogWrite(L_MOT,100);
	analogWrite(R_MOT,100);
	delay(2000);

	// Turn right on spot
	set_direction(1);
	analogWrite(L_MOT,100);
	analogWrite(R_MOT,100);
	delay(2000);

	// Turn left on spot
	set_direction(2);
	analogWrite(L_MOT,100);
	analogWrite(R_MOT,100);
	delay(2000);

	// Go backwards
	set_direction(3);
	analogWrite(L_MOT,100);
	analogWrite(R_MOT,100);
	delay(2000);

	// Stop 
	analogWrite(L_MOT,0);
	analogWrite(R_MOT,0);
	delay(2000);
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

