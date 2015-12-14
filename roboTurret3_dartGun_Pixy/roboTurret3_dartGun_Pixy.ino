/***********************************************************************************
 *
 *      __                 RobotGeek Foam Dart Gun 
 *       \_____                with Desktop RobotTurret and Pixy
 *       |      |          
 *       |______|=    =====>
 *         |  | 
 *        _|__|
 *       _|___|___
 *   _   |       |
 *  _|___|_______|_
 * 
 *  The following sketch will activate a single servo to fire the foam dart gun
 *  whenver a pushbutton is pressed.
 *
 *  ###ADD PIXY CONNECTIONS
 *  DIO 9 - Black RobotGeek 180 Degree Servo  White -'S' Black -'G'
 *  
 *  Use an external power supply and set the jumper for pins 9/10/11 to 'VIN'
 *   
 * This code is designed for a bare pushbutton and uses internal pullup resistors. 
 *
 *  For more information and wiring diagrams see
 *  http://learn.robotgeek.com/getting-started/29-desktop-roboturret/159-robotgeek-foam-dart-gun-getting-started-guide.html
 ***********************************************************************************/
 
#include <SPI.h>    //Serial Peripheral Interface for connecting to Pixy
#include <Pixy.h>   //Pixy Communication library
#include <Servo.h>  //include the servo library to control the RobotGeek Servos

Pixy pixy;    //create the pixy class

//Constant variables - these all use 'const' because they will not change during the program operation, allowing us to save RAM
const int TRIGGER_PIN = 3;  //Triffer Servo Digital Pin
const int PAN_PIN = 5;      //Pan Servo Digital Pin
const int TILT_PIN = 6;     //Tilt Servo Digital Pin
const int LASER_PIN = 2     //Laser Digital Pin

//max/min puse values in microseconds to send to the servo
//###FIRE_POSITION needs to change to microseconds
const int FIRE_POSITION = 113 //default position the servo will move to in order to fire the dar gun  
const int PAN_MIN = 600;  //full counterclockwise for RobotGeek 180 degree servo
const int PAN_MAX = 2400; //full clockwise for RobotGeek 180 degree servo
const int TILT_MIN = 600;     //full counterclockwise for RobotGeek 180 degree servo
const int TILT_MAX = 2400;    //full clockwise for RobotGeek 180 degree servo

const int TRACKING_TIMEOUT = 50; //time in milliseconds after object is lost that tracking stops
const int TIME_BFORE_FIRE = 5000;//time in milliseconds between first seeing the object and firing 

int speed = 4;        //alter this value to change the speed of the system. Higher values mean higher speeds 5-500 approximate recommended range

Servo panServo, tiltServo, triggerServo;  // create servo objects to control the pan and tilt servos

int panValue = 1500;   //current positional value being sent to the pan servo. 
int tiltValue = 1500;  //current positional value being sent to the tilt servo. 

int laserState = LOW;         //The current state of the laser module

int pixyCenterY;        //y coordinate of the block, centered 
int pixyCenterX;        //x coordinate of the block, centered
long pixySize;          //size of the block
long lastSeen;          //last time the pixy block was located
long trackStartTime;    //time that tracking started 
bool tracking = 0;      //whether or not the code is currently tracking an object
bool shots = 1;         //number of shots remaing - by default 1 for the foam dart gun

//setup servo objects and set initial position
void setup()
{ 
  
  panServo.attach(PAN_PIN, PAN_MIN, PAN_MAX);  // attaches/activates the pan servo on pin PAN_PIN and sets lower/upper limits that can be sent to the servo
  tiltServo.attach(TILT_PIN, TILT_MIN, TILT_MAX);  // attaches/activates the tilt servo on pin TILT_PIN and sets lower/upper limits that can be sent to the servo
  triggerServo.attach(TRIGGER_PIN); //attach the trigger servo on pin SERVOPIN

  //initalize digital pins
  pinMode(LASER_PIN, OUTPUT);      //set the LASER Pin to an output

  //the pixy class will initialize all of the pixy realated pins
  
  //write initial servo positions to set the servos to 'home'
  panServo.writeMicroseconds(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.writeMicroseconds(tiltValue);//sets the tilt servo position to the default 'home' value
  //### change to microseconds
  triggerServo.write(90);        //sets the servo position to 90 degress, centered
  Serial.begin(9600);
}
 
 
void loop()
{
  uint16_t blocks;  //block holds tracked data for the pixy
  char buf[32];     //pixy data buffer
  
  blocks = pixy.getBlocks();  

  //if blocks exist
  if(blocks)
  {
    lastSeen = millis();  //a block has been 'seen' so record the time
   
    //if the code wasnt' tracking before
    if(tracking == false)
    {
      trackStartTime = lastSeen;  //record tracking time start - this will be the same as lastSeen right now, but lastSeen will keep updated
      tracking = true;               //turn tracking flag on
      Serial.println("Start Tracking! Time"");
      Serial.println(trackStartTime);
    }
    
    pixyCenterX = pixy.blocks[0].x - 160; //get x data (0-320) and center it by subtracting 160. Values 0-159 (to the left) will be negative while values 161-320 (to the right) will be positive. This will help in tracking objects
    pixyCenterY = pixy.blocks[0].y - 100; //get y data (0-200) and center it by subtracting 100. Values 0-99 (down) will be negative while values 101-200 (up) will be positive. This will help in tracking objects
    pixySize = pixy.blocks[0].height *  pixy.blocks[0].width; //get the relative size of the object by multiplying width * height
  
    panValue = panValue - pixyCenterX/speed;
    tiltValue = tiltValue - pixyCenterY/speed; 
  
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //panValue variable within the min/max bounds, or the turret may become unresponsive
     panValue = max(panValue, PAN_MIN);  //use the max() function to make sure the value never falls below PAN_MIN (0 degrees)
     panValue = min(panValue, PAN_MAX);  //use the min() function to make sute the value never goes above PAN_MAX (180 degrees)
   
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //tiltValue variable within the min/max bounds, or the turret may become unresponsive
     tiltValue = max(tiltValue, TILT_MIN);//use the max() function to make sure the value never falls below 0
     tiltValue = min(tiltValue, TILT_MAX);//use the min() function to make sute the value never goes above 180
     
     panServo.writeMicroseconds(panValue);  //sets the pan servo position to the default 'home' value
     tiltServo.writeMicroseconds(tiltValue);//sets the tilt servo position to the default 'home' value

     //if the object has been tracked for at least TIME_BFORE_FIRE millieseconds, and the dart gun has a shot, proceed
     if((millis() - trackStartTime  > TIME_BFORE_FIRE) && (shots > 0))
     {
      Serial.print("FIRE!");
      triggerServo.write(FIRE_POSITION); // move the servo to FIRE_POSITION to fire the dart gun
      delay(300);                        //wait for 300ms for servo to move
      triggerServo.write(90);            // sets the servo position to 90 degress, centered
      shots = shots - 1;                  //decrement the shot counter so the gun doesn't keep trying to fire
     }
     
  }
  //if no blocks exist
  else
  {
    //check if it has beeen TRACKING_TIMEOUT milliseconds before setting the tracking flag false - this helps to mitigate lost data packets or short connectiviety losses
    if(millis()-lastSeen > TRACKING_TIMEOUT)
    {
      tracking = false; 
      Serial.println("Object Lost, no longer tracking");
    }
  }

  //###DELETE THIS before going live
//  //read the pushbutton pin. Because the code is using internal pullups, a 'LOW' signal indicated the button has been pressed
//  if (HIGH == LOW)
//  {  
//    triggerServo.write(FIRE_POSITION); // move the servo to FIRE_POSITION to fire the dart gun
//    delay(300);                        //wait for 300ms
//    triggerServo.write(90);            // sets the servo position to 90 degress, centered
//    delay(300);                        //wait for 300ms before next possible firing attempt    
//  }
  
  
}
 
 
 
 
 
