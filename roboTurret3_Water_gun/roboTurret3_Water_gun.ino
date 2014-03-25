/***********************************************************************************
 *     ______            RobotGeek Desktop RoboTurret v3         ______
 *      |  |            Direct Joystick & Button Control          |  | 
 *      |__|_                                                    _|__|
 *   ___|____|_                                                 _|___|___
 *    |       |    _                                        _   |       |
 *   _|_______|____|_                                      _|___|_______|_
 * 
 *  The following sketch will allow you to control a Desktop RobotTurret v3 using
 *  the included RobotGeek Joysticl and RobotGeek Pushbutton
 *    http://www.trossenrobotics.com/robotGeek-pushbutton    
 *    http://www.trossenrobotics.com/robotgeek-joystick  
 *    
 *  Wiring
 *    Pan Servo - Digital Pin 10 
 *    Tilt Servo - Digital Pin 11 
 *    Laser - Digital Pin 2
 *    Pushbutton Servo - Digital Pin 4 
 *    Joystick(Horizontal)- Analog Pin 0 
 *    Joystick(Vertica)l- Analog Pin 2
 *    Jumper for pins 9/10/11 should be set to 'VIN'
 *
 *  Control Behavior:
 *    The Horizontal Joystick will move the Pan Servo
 *    The Vertical Joystick will move the Tilt Servo
 *    The Pushbutton  will toggle the laser mofule on/off
 *
 *  External Resources
 *
 ***********************************************************************************/
//Includes
#include <Servo.h> 

//Defines
#define PAN_PIN 10        //Pan Servo Digital Pin
#define TILT_PIN 11       //Tilt Servo Digital Pin

#define H_JOY_PIN 0       //Horizontal Joystick Analog Pin
#define V_JOY_PIN 1       //Vertical Joystick Analog Pin 

#define LASER_PIN 7       //Laser Digital Pin

#define PUSHBUTTON_PIN 4  //Pushbutton Digital Pin

//deadband values for the joysticks - values between DEADBANDLOW and DEADBANDHIGH will be ignored
#define DEADBANDLOW 480   //lower deadband value for the joysticks  
#define DEADBANDHIGH 540  //upper deadband value for the joysticks  

//max/min puse values in microseconds to send to the servo
#define PAN_MIN      600  //full counterclockwise for RobotGeek 180 degree servo
#define PAN_MAX      2400 //full clockwise for RobotGeek 180 degree servo
#define TILT_MIN  600     //full counterclockwise for RobotGeek 180 degree servo
#define TILT_MAX  1000    //full clockwise for RobotGeek 180 degree servo

int speed = 75;        //alter this value to change the speed of the system. Higher values mean higher speeds 5-500 approximate recommended range

Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos

int horizontalValue, verticalValue;            //variables to hold the last reading from the analog pins for the horizontal and vertical joystick
int horizontalValueMapped, verticalValueMapped;//variables to hold mapped readings from the vertical values. These mapped readings will be appropriate to work with servo values

int panValue = 1500;   //current positional value being sent to the pan servo. 
int tiltValue = 1000;  //current positional value being sent to the tilt servo. 

//State Variables
int laserState = HIGH;         //The current state of the laser module
int buttonState;             // the current state of the pushbuton
int lastButtonState = HIGH;   // the previous reading from the input pin

//Timing variables for button debouncing
long lastDebounceTime = 0;  // the last time the output pin was toggled. This variable is a 'long' because it may need to hold many milliseconds, and a 'long' will afford more space than an 'int'
int debounceDelay = 50;    // the amount of time that that a button must be held, for a reading to register (in milliseconds)


void setup() 
{ 
  //initialize servos
  panServo.attach(PAN_PIN, PAN_MIN, PAN_MAX);  // attaches/activates the pan servo on pin PAN_PIN and sets lower/upper limits that can be sent to the servo
  tiltServo.attach(TILT_PIN, TILT_MIN, TILT_MAX);  // attaches/activates the tilt servo on pin TILT_PIN and sets lower/upper limits that can be sent to the servo

  //initalize digital pins
  pinMode(PUSHBUTTON_PIN, INPUT);  //set the PUSHBUTTON Pin to an Input
  pinMode(LASER_PIN, OUTPUT);      //set the LASER Pin to an output
  
  //Analog pins do not need to be initialized
  
  //write initial servo positions to set the servos to 'home'
  panServo.write(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.write(tiltValue);//sets the tilt servo position to the default 'home' value
  
} 

void loop() 
{ // read the state of the pushbutton value:
  buttonState = digitalRead(PUSHBUTTON_PIN);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {     
    // turn LED on:    
    digitalWrite(LASER_PIN, HIGH);  
  } 
  else {
    // turn LED off:
    digitalWrite(LASER_PIN, LOW); 
  
  }
  
   
  
    /**************Servo Positions *******************************/
  //read the values from the analog sensors/joysticks
   horizontalValue = analogRead(H_JOY_PIN);
   verticalValue = analogRead(V_JOY_PIN);
   
   //check that the joystick is outisde of the deadband. Movements in the deadband should not register
   if(horizontalValue < DEADBANDLOW || horizontalValue > DEADBANDHIGH)
   {
     //horizontalValue will hold a value between 0 and 1023 that correspods to the location of the joystick. The map() function will convert this value
     //into a value between speed and -speed. This value can then be added to the current panValue to incrementley move ths servo 
     horizontalValueMapped = map(horizontalValue, 0, 1023, -speed, speed) ;
     
     panValue = panValue + horizontalValueMapped; //add the horizontalValueMapped to panValue to slowly increment/decrement the tiltValue
     
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //panValue variable within the min/max bounds, or the turret may become unresponsive
     panValue = max(panValue, PAN_MIN);  //use the max() function to make sure the value never falls below PAN_MIN (0 degrees)
     panValue = min(panValue, PAN_MAX);  //use the min() function to make sute the value never goes above PAN_MAX (180 degrees)


   }   
   
   //check that the joystick is outisde of the deadband. Movements in the deadband should not register
   if(verticalValue < DEADBANDLOW || verticalValue > DEADBANDHIGH)
   {
     //horizontalValue will hold a value between 0 and 1023 that correspods to the location of the joystick. The map() function will convert this value
     //into a value between speed and -speed. This value can then be added to the current panValue to incrementley move ths servo 
     verticalValueMapped = map(verticalValue, 0, 1023, -speed, speed) ;
     
     tiltValue = tiltValue + verticalValueMapped; //add the verticalValueMapped to tiltValue to slowly increment/decrement the tiltValue
          
     //even though the servos have min/max value built in when servo.attach() was called, the program must still keep the
     //tiltValue variable within the min/max bounds, or the turret may become unresponsive
     tiltValue = max(tiltValue, TILT_MIN);//use the max() function to make sure the value never falls below 0
     tiltValue = min(tiltValue, TILT_MAX);//use the min() function to make sute the value never goes above 180
   }
  panServo.writeMicroseconds(panValue);   // sets the servo position based on the latest panServo value
  tiltServo.writeMicroseconds(tiltValue); // sets the servo position based on the latest tiltServo value
   
  delay(15); // waits for the servo to get to they're position before proceeding
} 

