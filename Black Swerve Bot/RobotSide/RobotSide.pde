#include <Ethernet.h>
#include <Servo.h>
#include <ClientUDP.h>

byte mac[] = { 0x00, 0x1D, 0x60, 0xAF, 0x03, 0x32 };
byte ip[]  = { 192, 168, 1, 22 };
byte gw[] = { 192, 168, 1, 1 };
byte subnet[] = { 255, 255, 255, 0 };

/* PID Constants */
#define frontMIN 1070
#define frontMAX 0
#define rearMIN 970
#define rearMAX -100
#define PCONSTANT 5

/* Control Constants */
#define JOYSTICK_TRIM_MAX 130
#define JOYSTICK_TRIM_MIN 80
#define TURNING_MAX_ERROR 30
#define TURNING_MIN_ERROR -30


#define PORT  4444
byte ip_from[] = {0,0,0,0};
uint16_t port_from = 0;
uint16_t pkt_len;
byte joystick_data[8];

ClientUDP udp(ip_from,port_from);

/* Robot Specific Stuff */
boolean robotEnabled = false;    //Robot Enable/Disable Killswitch
unsigned long prevLastUpdate = 0; //Keeps track of prior last update for latency debugging
unsigned long lastUpdate = 0;  //Keeps track of the last time (ms) we received data

//Define robot outputs
int pwm01 = 3;
int pwm02 = 9;
int pwm03 = 5;
int pwm04 = 6;

//POT Tracking vars
int lastFrontPOT = 512;
int lastRearPOT = 512;
int frontPOT = 512;
int rearPOT = 512;
int frontPotTurns = 0;
int rearPotTurns = 0;

//Speed Controller Objects
Servo frontRotation;
Servo rearRotation;
Servo leftDrive;
Servo rightDrive;


void setup() {
  Ethernet.begin(mac, ip, gw, subnet);
  Serial.begin(9600);
  
  //Enable the robot
  robotEnabled = true;
  
  Serial.println("Robot Control Platform v0.1 Initialized");
  udp.open(PORT);
}


void loop() {
    xferdata();
    
    frontPOT = analogRead(0);
    rearPOT = analogRead(1);
    
    if ((frontPOT - lastFrontPOT) > 500)
      frontPotTurns -= 1;
    if ((frontPOT - lastFrontPOT) < -500)
      frontPotTurns += 1;
      
    if ((rearPOT - lastRearPOT) > 500)
      rearPotTurns -= 1;
    if ((rearPOT - lastRearPOT) < -500)
      rearPotTurns += 1;
      
    lastFrontPOT = frontPOT;
    lastRearPOT = rearPOT;
  
    //Only allow robot to be enabled if we've received data in the last 500ms and robot is set to enabled
    if (robotEnabled && (millis() - lastUpdate) <= 100 && millis() > 500)
      enabled();
    else
      disabled();
      
    
}


void xferdata()
{
  if (udp.available()) {
    if (pkt_len = udp.read(ip_from,&port_from,joystick_data,sizeof(joystick_data))) {
      if (pkt_len > 1) {
        prevLastUpdate = lastUpdate;
        lastUpdate = millis();  //Keep track of the last time we received data
      }
    }
  }
}



/*
joystick 0 - left y
joystick 1 - left x
joystick 2 - right y
joystick 3 - right x
*/


void enabled()
{	
    frontRotation.attach(pwm01);
    rearRotation.attach(pwm02);
    leftDrive.attach(pwm03);
    rightDrive.attach(pwm04);
   
    frontPOT += frontPotTurns * 1023;
    rearPOT += rearPotTurns * 1023;
  

    /*if (joystick_data[4] == 0) {  //NEW CRAB DRIVE
      int steeringDirection = joystick_data[3];
      if (joystick_data[2] < 127)    //If we are moving in reverse we need to spin the wheels to the opposite angle
        steeringDirection = 255 - steeringDirection;
    
      int frontError = frontPOT - map(steeringDirection, 0, 255, frontMIN, frontMAX);  //Find error for front wheels
      int rearError = rearPOT - map(steeringDirection, 0, 255, frontMIN, frontMAX);   //Find error for rear wheels
      
      driveRotation(frontError, rearError);
      
      int joystickY = joystick_data[2] - 127;
      int joystickX = joystick_data[3] - 127;
      
      int magnitude = sqrt(joystickX*joystickX + joystickY*joystickY);
      
      magnitude = map(magnitude, 0, 180, 127, 0);
      
      if (joystick_data[2] < 127)  //go in reverse
        driveMotors(magnitude, magnitude + 33);
      else
        driveMotors((255 - magnitude), (255 - magnitude) + 33);
    }*/
    if (joystick_data[4] == 0) {  //OLD CRAB DRIVE
      int frontError = frontPOT - map(joystick_data[3], 0, 255, frontMIN, frontMAX);  //Find error for front wheels
      int rearError = rearPOT - map(joystick_data[3], 0, 255, frontMIN, frontMAX);   //Find error for rear wheels
      
      driveRotation(frontError, rearError);
          
      driveMotors(joystick_data[0], joystick_data[0]);
    }
    else if (joystick_data[4] == 1) {  //CAR DRIVE
      int frontError = frontPOT - map(joystick_data[3], 0, 255, 212, 812);        //Find error for front wheels
      int rearError = rearPOT - map(255 - joystick_data[3], 0, 255, 212, 812);   //Find error for rear wheels (inverted for car drive)
       
      driveRotation(frontError, rearError);
        
      driveMotorsCAR(joystick_data[0], joystick_data[0]);  
    }
    else if (joystick_data[4] == 2) {  //TANK DRIVE
      int frontError = frontPOT - 512;  //Keep front wheels centered
      int rearError = rearPOT - 512;   //Keep rear wheels centered
      
      driveRotation(frontError, rearError);
          
      driveMotors(joystick_data[2], joystick_data[0]);
    }
}


void disabled()
{
	leftDrive.detach();
	rightDrive.detach();
        frontRotation.detach();
	rearRotation.detach();
}







/***** DRIVE FUNCTIONS *****/
/***** DRIVE FUNCTIONS *****/
/***** DRIVE FUNCTIONS *****/

/* This function handles the rotation of the wheels for steering given the current error */
void driveRotation(int frontError, int rearError) {
    int rawFrontError = frontError;
    int rawRearError = rearError; 
                
    frontError *= PCONSTANT;
    rearError *= PCONSTANT;
    
    if (frontError > 511)
      frontError = 511;
    else if (frontError < -512)
      frontError = -512;
    if (rearError > 511)
      rearError = 511;
    else if (rearError < -512)
      rearError = -512;    
          
    if (rawFrontError > TURNING_MAX_ERROR || rawFrontError < TURNING_MIN_ERROR)
      frontRotation.write(map(frontError, -512, 511, 0, 180));
    else
      frontRotation.write(90);
        
    if (rawRearError > TURNING_MAX_ERROR || rawRearError < TURNING_MIN_ERROR)
      rearRotation.write(map(rearError, -512, 511, 0, 180));
    else
      rearRotation.write(90);

}


/* Used for left/right drive in crab and tank drive mode */
void driveMotors(int joystickLeft, int joystickRight) {
      joystickLeft = map(255 - joystickLeft, 0, 255, 0, 180);
      joystickRight = map(joystickRight, 0, 255, 0, 180);
      
      if (joystickLeft < JOYSTICK_TRIM_MAX && joystickLeft > JOYSTICK_TRIM_MIN)
        joystickLeft = 90;
      if (joystickRight < JOYSTICK_TRIM_MAX && joystickRight > JOYSTICK_TRIM_MIN)
        joystickRight = 90;
      
      leftDrive.write(joystickLeft);
      rightDrive.write(joystickRight);
}



/* Used for left/right drive in car mode */
/* This function has to vary the speed of the left/right drives based on turning angle */
void driveMotorsCAR(int joystickLeft, int joystickRight) {
      //If the throttle is not being pushed on, don't let the drives move
      Serial.println(joystickLeft);
      if (joystickLeft < JOYSTICK_TRIM_MAX && joystickLeft > JOYSTICK_TRIM_MIN) {
        leftDrive.write(90);
        rightDrive.write(90);
      }
      else {
        //Drive code
        joystickLeft = map(255 - joystickLeft, 0, 255, 0, 180);
        joystickRight = map(joystickRight, 0, 255, 0, 180);
        
        int Turning = map(analogRead(0), frontMIN, frontMAX, 0, 180);
        
        int leftDrivePwr = joystickLeft;
        int rightDrivePwr = joystickRight;
        
        if (Turning >= 90) { //we are turning right
            if (rightDrivePwr >= 90)
              rightDrivePwr -= map(Turning, 90, 180, 0, 90);
            else if (rightDrivePwr < 90)
              rightDrivePwr += map(Turning, 90, 180, 0, 90);
        }
        else if (Turning < 90) {  //turning left
            if (leftDrivePwr >= 90)
              leftDrivePwr -= map(Turning, 90, 0, 0, 90);
            else if (leftDrivePwr < 90)
              leftDrivePwr += map(Turning, 90, 0, 0, 90);
        }
        
        
        if (leftDrivePwr < JOYSTICK_TRIM_MAX && leftDrivePwr > JOYSTICK_TRIM_MIN)
          leftDrivePwr = 90;
        if (rightDrivePwr < JOYSTICK_TRIM_MAX && rightDrivePwr > JOYSTICK_TRIM_MIN)
          rightDrivePwr = 90;
        
        leftDrive.write(leftDrivePwr);
        rightDrive.write(rightDrivePwr);
      }
}
