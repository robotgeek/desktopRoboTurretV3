/***********************************************************************************
 *     ______            RobotGeek Desktop RoboTurret v3         ______
 *      |  |                    LazerDazer                        |  | 
 *      |__|_                                                    _|__|
 *   ___|____|_                                                 _|___|___
 *    |       |    _                                        _   |       |
 *   _|_______|____|_                                      _|___|_______|_
 * 
 *  The following sketch will turn the laser on and randomly move the pan and tilt
 *  servos.
 *    
 *  Wiring:
 *    Pan Servo - Digital Pin 10 
 *    Tilt Servo - Digital Pin 11 
 *    Laser - Digital Pin 2
 *   
 *  Control Behavior:
 *    No inputs. Plug it in and it will run.
 *   
 *  External Resources:
 *    https://www.arduino.cc/en/Reference/Random
 *   https://www.arduino.cc/en/Reference/RandomSeed
 *    https://www.arduino.cc/en/Reference/Servo
 *
 ***********************************************************************************/
//Includes
#include <Servo.h>          //include the servo library for working with servo objects

//Defines
const int PAN = 10;         //Pan Servo Digital Pin
const int TILT = 11;        //Tilt Servo Digital Pin
const int LASER = 2;        //Laser Digital Pin


const int PAN_MIN = 40;     // minimum pan servo range, value should be between 0 and 180 and lower than PAN_MAX
const int PAN_MAX = 140;    // maximum pan servo range, value should be between 0 and 180 and more than PAN_MIN

const int TILT_MIN = 60;    // minimum tilt servo range, value should be between 0 and 180 and lower than TILT_MAX
const int TILT_MAX = 100;   // maximum tilt servo range, value should be between 0 and 180 and more than TILT_MIN

const int SPEED_MIN = 10;   // minimum interpolation speed (less than 5 not recommended)
const int SPEED_MAX = 10;   // maximum interpolation speed (Lower SPEED_MAX values mean that the program will cycle more quickly, leading to an increased occurance of pausing. Adjust ROLL_MAX accordingly)
                       
const int ROLL_MAX = 40;    // Roll the dice for random pause. Lower value = more frequent pauses, higher value = less frequent pauses

const int PAUSE_MIN = 500;  // define minimum pause length in mS. 1000 = 1 second
const int PAUSE_MAX = 1500; // define maximum pause length in mS.


Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos

long dazer_speed;
int dazer_pause_roll;
int pause_length = 1;


int panValue = 90;   //current positional value being sent to the pan servo. 
int tiltValue = 90;  //current positional value being sent to the tilt servo. 
byte pan_goal;
byte tilt_goal;


//State Variables
int laserState = LOW;         //The current state of the laser module

//Timing variables
long lastDebounceTime = 0;  // the last time the output pin was toggled. This variable is a 'long' because it may need to hold many milliseconds, and a 'long' will afford more space than an 'int'
int debounceDelay = 50;    // the amount of time that that a button must be held, for a reading to register (in milliseconds)

void setup() 
{ 
  
  //initialize servos
  panServo.attach(PAN);  // attaches/activates the pan servo on pin PAN 
  tiltServo.attach(TILT);  // attaches/activates the tilt servo on pin TILT 

  //initalize digital pins
  pinMode(LASER, OUTPUT);      //set the LASER Pin to an output
  
  //write initial servo positions to set the servos to 'home'
  panServo.write(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.write(tiltValue);//sets the tilt servo position to the default 'home' value
  
  randomSeed(analogRead(0));  //set a basis for the random() engine, based off of analog0 which will be a random number because nothing is attached

  digitalWrite(LASER, HIGH);
} 

void loop() 
{ 
  dazer_pause_roll = random(1, ROLL_MAX);      //pick a random number between 1 and ROLL_MAX
  pause_length = random(PAUSE_MIN, PAUSE_MAX); //pick a random number between PAUSE_MIN and PAUSE_MAX
  delay(pause_length);                         //use the random number to pause for pause_length milliseconds

  pan_goal = random(PAN_MIN, PAN_MAX);
  tilt_goal = random(TILT_MIN, TILT_MAX);
  dazer_speed = random(SPEED_MIN, SPEED_MAX);
  //perfom the loop while the current positions are different from the goal positions 

  //if the pan_goal is larger than the pan position, move the pan position up
  if (pan_goal > panValue)
  {
    panValue = panValue++;
  }
  
  //if the pan_goal is smaller than the pan position, move the pan position down
  else if (pan_goal < panValue)
  {
    panValue = panValue--;
  }               
  
  //if the tilt_goal is larger than the tilt position, move the pan position up
  if (tilt_goal > tiltValue)
  {
    tiltValue = tiltValue++;
  }
  //if the tilt_goal is smaller than the tilt position, move the pan position down
  else if (tilt_goal < tiltValue)
  {
    tiltValue = tiltValue--;
  }    
  
  panServo.write(pan_goal);                    // sets the servo position according to the scaled value 
  tiltServo.write(tilt_goal);                  // sets the servo position according to the scaled value 
  delay(dazer_speed);                          // waits for a random delay before the next loop
} //end loop()




