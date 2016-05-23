 /* 
   * Pan/Tilt Commander Incremental Control
   
   *ARBOTIX COMMANDER CONTROLS **********
   *Left Joystick
   **Control the pan and tilt servos at a higher speed. Pressing the left/right top buttons allows for speed control.
   **Left-Right = Pan     
   **Up-Down = Tilt
   *
   *Right Joystick
   **Control the pan and tilt servos at a slower, finer speed.
   **Left-Right = Pan       
   **Up-Down = Tilt
   *
   *Left Pushbuttons (L4-L6)
   **Set the tilt servo to 3 predetermined positions, the lower limit, defualt position, and upper limit
   *
   *Right Pushbuttons(R1-R3)
   **Set the pan servo to 3 predetermined positions, the lower limit, defualt position, and upper limit
   *
   *Left Top Button
   **Turn off the Laser
   *  
   *Right Top Button   
   **Turn on the Laser
   ********************* 
  ArbotiX Firmware - Commander Extended Instruction Set Example
  Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 
 
 
//define pan and tilt servo IDs
#define PAN_PIN 10        //Pan Servo Digital Pin
#define TILT_PIN 11       //Tilt Servo Digital Pin

#define LASER_PIN 2       //Laser Digital Pin

// the H101 'C' bracket attached to the tilt servo creates a physical limitation to how far
// we can move the tilt servo. This software limit will ensure that we don't jam the bracket into the servo.
#define TILT_UPPER_LIMIT 2400
#define TILT_LOWER_LIMIT 600

//Upper/Lower limits for the pan servo - by defualt they are the normal 0-4095 (0-360) positions for the servo
#define PAN_UPPER_LIMIT 2400 
#define PAN_LOWER_LIMIT 600

//Default/Home position. These positions are used for both the startup position as well as the 
//position the servos will go to when they lose contact with the commander
#define DEFAULT_PAN 1500
#define DEFAULT_TILT 1500

#define WALKDEADBAND 20 //the 'walk' joystick (left) has a range from -128 to 127. Define a deadband(-20/+20) to reduce drift/stutter.

#define HOME_ON_DISCONNECT false //set to 'true' and the turret will return to the defualt position when it loses contact with the commander. Set to 'false' and the turret will remain in the position it was last in 
#define ARBOTIX_TIMEOUT  1000      // time(ms) after the commander stops sending messages for the turret to return to a defult position.

 
//Include necessary Libraries to drive the servos and recieve commander instructions  
#include <Commander.h>
#include <Servo.h> 

Commander command = Commander(); //start commander object

int pan;    //current position of the pan servo
int tilt;   //current position of the tilt servo  
int panMod  = 1;  //modifier for pan speed on left joystick - increase this to reduce the turret's speed
int tiltMod = 1;  //modifier for tilt speed on left joystick - increase this to reduce the turret's speed
int panSpeed;  //increment value for pan axis
int tiltSpeed;  //increment value for tilt axis
Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos

boolean turretActive = false;   // Is the turret on/receieivng commands - used to set the turret to a defualt position if it loses contact with the commander
unsigned long   lastMsgTime;    // Keep track of when the last message arrived to see if controller off

boolean leftPrev = false;      //left top button previous state
boolean rightPrev = false;     //right top button previous state

unsigned long lastUpdateTime;
int updateInterval = 33;

void setup(){
  //initialize servos
  panServo.attach(PAN_PIN, PAN_LOWER_LIMIT, PAN_UPPER_LIMIT);  // attaches/activates the pan servo on pin PAN_PIN and sets lower/upper limits that can be sent to the servo
  tiltServo.attach(TILT_PIN, TILT_LOWER_LIMIT, TILT_UPPER_LIMIT);  // attaches/activates the tilt servo on pin TILT_PIN and sets lower/upper limits that can be sent to the servo

  pinMode(LASER_PIN, OUTPUT);      //set the LASER Pin to an output

  // setup serial for commander communications 
  Serial.begin(38400);
  
  // setup interpolation, slowly raise turret to a 'home' positon. 2048 are the 'center' positions for both servos
  pan = DEFAULT_PAN;
  tilt = DEFAULT_TILT;
  
  delay(100);//short delay
  
  //write initial servo positions to set the servos to 'home'
  panServo.writeMicroseconds(pan);  //sets the pan servo position to the default 'home' value
  tiltServo.writeMicroseconds(tilt);//sets the tilt servo position to the default 'home' value
  
  lastMsgTime = millis();    // log time, to calculate  disconnect time
}
 
void loop(){
  // try to read a command from commander
  if(command.ReadMsgs() > 0)
  {
      turretActive = true;//if a command has been received, 'activate' the turret
      
      // toggle LEDs
      digitalWrite(0,HIGH-digitalRead(0));
      
      //set pan /tilt based on walk/left joystick, by defualt moves quickly
      //set pan /tilt based on look/right joystick, moves slowly for fine control
      //the walk/left joystick needs a deadband around the center because it is issuing finer movements. Without a deadband
      //the turret will stutter and drift
      //create a deadband around the left joystick by comapring the absolute value (command.walk's range is -128 to 127) to the deadband
      if(abs(command.walkH) > WALKDEADBAND)
      { 
        panSpeed = command.walkH/panMod; //use 'panMod' to adjust the speed of the pan servo
      }
      else if(abs(command.walkH) < WALKDEADBAND && abs(command.lookH) < WALKDEADBAND)
      {
        panSpeed = 0;
      }
      if(abs(command.walkV) > WALKDEADBAND)
      {
        tiltSpeed =  command.walkV/tiltMod;  //use 'tiltMod' to adjust the speed of the tilt servo
      }
      else if(abs(command.walkV) < WALKDEADBAND && abs(command.lookV) < WALKDEADBAND)
      {
        tiltSpeed = 0;
      }
      
      if(abs(command.lookH) > WALKDEADBAND)
      { 
        panSpeed = command.lookH/(panMod+9);  //use 'panMod' to adjust the speed of the pan servo
      }
      else if(abs(command.lookH) < WALKDEADBAND && abs(command.walkH) < WALKDEADBAND)
      {
        panSpeed = 0;
      }
      if(abs(command.lookV) > WALKDEADBAND)
      {
        tiltSpeed =  command.lookV/(tiltMod+9);  //use 'tiltMod' to adjust the speed of the tilt servo
      }
      else if(abs(command.lookV) < WALKDEADBAND && abs(command.walkV) < WALKDEADBAND)
      {
        tiltSpeed = 0;
      }
  
    
      //set pre-defined positions based on pushbuttons R1-L6
      if(command.buttons & BUT_R1)
      {
        pan = PAN_LOWER_LIMIT;
      }
      else if(command.buttons & BUT_R2)
      {
        pan = DEFAULT_PAN;
      }
      else if(command.buttons & BUT_R3)
      {
        pan = PAN_UPPER_LIMIT;
      }
      else if(command.buttons & BUT_L4)
      {
         tilt = TILT_UPPER_LIMIT;
      }
      else if(command.buttons & BUT_L5)
      {
        tilt = DEFAULT_TILT;
      }
      else if(command.buttons & BUT_L6)
      {
         tilt = TILT_LOWER_LIMIT;
      }
      //alter the on/off state of the laser pin with the right and left shoulder trigger buttons
      else if(command.buttons & BUT_RT)
      {
        digitalWrite(LASER_PIN, HIGH);
      }

      else if(command.buttons & BUT_LT)
      {
        digitalWrite(LASER_PIN, LOW);
      }      
      
      //enforce upper/lower limits for tilt servo
      if (tilt < TILT_LOWER_LIMIT)
      {
        tilt =TILT_LOWER_LIMIT;
      }  
    
      else if (tilt > TILT_UPPER_LIMIT)
      {
        tilt =TILT_UPPER_LIMIT;
      }
    
    
      //enforce upper/lower limits for pan servo
      if (pan < PAN_LOWER_LIMIT)
      {
        pan =PAN_LOWER_LIMIT;
    
      }  
    
      else if (pan > PAN_UPPER_LIMIT)
      {
        pan =PAN_UPPER_LIMIT;
    
      }
      
      lastMsgTime = millis();    // log the current time of the last command succesfully received and executed
  }    
  
  
  else //no commands from commander
  {

      // check to see if...
      //HOME_ON_DISCONNECT is 'true', meaning the turret should return to the home position after it stops receiving commands
      //turretActive is 'true' meaning the turret has received commands in the past
      //the program has exceeded the timeout, ARBOTIX_TIMEOUT milliseconds, i.e. the turret has not received a command in that time period
      // If these have all occured, set the turret to its home position
      if (HOME_ON_DISCONNECT &&(turretActive) && ((millis() - lastMsgTime) > ARBOTIX_TIMEOUT)) 
      {
        turretActive = false;//deactivate turret
        //set defualt servo positions
        pan = DEFAULT_PAN;
        tilt = DEFAULT_TILT;
        
        delay(100);//short delay

      }    
  }



  //update the servos regularly with the proper pan and tilt increments
  if(millis() - lastUpdateTime > updateInterval)
  {
    pan -= panSpeed;//use 'panMod' to adjust the speed of the pan servo
    tilt -= tiltSpeed;//use 'tiltMod' to adjust the speed of the tilt servo. Change "-=" to "+=" to invert the movement axis.
    pan = constrain(pan, PAN_LOWER_LIMIT, PAN_UPPER_LIMIT);
    tilt = constrain(tilt, TILT_LOWER_LIMIT, TILT_UPPER_LIMIT);
    panServo.writeMicroseconds(pan);   // sets the servo position based on the latest panServo value
    tiltServo.writeMicroseconds(tilt); // sets the servo position based on the latest tiltServo value
    lastUpdateTime = millis();
  }
}

