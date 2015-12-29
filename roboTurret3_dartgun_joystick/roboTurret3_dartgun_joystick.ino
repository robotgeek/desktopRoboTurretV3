/***********************************************************************************
 *
 *      __                 RobotGeek RobotTurret and Foam Dart Gun Test
 *       \_____ 
 *       |      |          
 *       |______|=    =====>
 *         |  | 
 *        _|__|
 *       _|___|___
 *   _   |       |
 *  _|___|_______|_
 * 
 *  The following sketch will allow you to control a Desktop RobotTurret v3 connected 
 *  to a foam dart gun using the included RobotGeek Joystick and RobotGeek Pushbutton
 *  
 *  The laser module will be turned on by defualt to allow for tracking. Do not aim
 *  the dart gun at animals or people.
 *    
 *  Wiring
 *    Trigger Servo - Digital Pin 3
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
 *    The Pushbutton  will fire the dart gun
 *
 *  External Resources
 *
 ***********************************************************************************/
//Includes
#include <Servo.h> 

//##change these to const int
//Defines
#define PAN_PIN 10        //Pan Servo Digital Pin
#define TILT_PIN 11       //Tilt Servo Digital Pin

#define H_JOY_PIN 0       //Horizontal Joystick Analog Pin
#define V_JOY_PIN 1       //Vertical Joystick Analog Pin 

#define LASER_PIN 2       //Laser Digital Pin

#define PUSHBUTTON_PIN 4  //Pushbutton Digital Pin

//deadband values for the joysticks - values between DEADBANDLOW and DEADBANDHIGH will be ignored
#define DEADBANDLOW 480   //lower deadband value for the joysticks  
#define DEADBANDHIGH 540  //upper deadband value for the joysticks  

//max/min puse values in microseconds to send to the servo
#define PAN_MIN      600  //full counterclockwise for RobotGeek 180 degree servo
#define PAN_MAX      2400 //full clockwise for RobotGeek 180 degree servo
#define TILT_MIN  600     //full counterclockwise for RobotGeek 180 degree servo
#define TILT_MAX  2400    //full clockwise for RobotGeek 180 degree servo
//##change this to ms
#define FIRE_POSITION 113 //default position the servo will move to in order to fire the dart gun  

int speed = 10;        //alter this value to change the speed of the system. Higher values mean higher speeds 5-500 approximate recommended range

Servo panServo, tiltServo, triggerServo;  // create servo objects to control the trigger, pan and tilt servos

int horizontalValue, verticalValue;            //variables to hold the last reading from the analog pins for the horizontal and vertical joystick
int horizontalValueMapped, verticalValueMapped;//variables to hold mapped readings from the vertical values. These mapped readings will be appropriate to work with servo values

int panValue = 1500;   //current positional value being sent to the pan servo. 
int tiltValue = 1500;  //current positional value being sent to the tilt servo. 

//State Variables
int laserState = LOW;         //The current state of the laser module
int buttonState;             // the current state of the pushbuton
int lastButtonState = LOW;   // the previous reading from the input pin

//Timing variables for button debouncing
long lastDebounceTime = 0;  // the last time the output pin was toggled. This variable is a 'long' because it may need to hold many milliseconds, and a 'long' will afford more space than an 'int'
int debounceDelay = 50;    // the amount of time that that a button must be held, for a reading to register (in milliseconds)

void setup() 
{ 
  //initialize servos
  panServo.attach(PAN_PIN, PAN_MIN, PAN_MAX);  // attaches/activates the pan servo on pin PAN_PIN and sets lower/upper limits that can be sent to the servo
  tiltServo.attach(TILT_PIN, TILT_MIN, TILT_MAX);  // attaches/activates the tilt servo on pin TILT_PIN and sets lower/upper limits that can be sent to the servo
  //##figure out max/min for trigger servo
  triggerServo.attach(SERVOPIN); //attach the trigger servo on pin SERVOPIN
  triggerServo.write(90);        //sets the servo position to 90 degress, centered
  
  //initalize digital pins
  pinMode(PUSHBUTTON_PIN, INPUT);  //set the PUSHBUTTON Pin to an Input
  pinMode(LASER_PIN, OUTPUT);      //set the LASER Pin to an output
  
  //Analog pins do not need to be initialized
  
  //write initial servo positions to set the servos to 'home'
  panServo.writeMicroseconds(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.writeMicroseconds(tiltValue);//sets the tilt servo position to the default 'home' value
  
} 

void loop() 
{ 
  
  /**************Button Reading and Debouncing / Laser Control *******************************/
  //In this sketch the Pushbutton will be used to toggle the laser on and off.
  //When a user makes/breaks electrical contacts by pushing/releasing the pushbutton,
  //the signal can 'bounce' between LOW and HIGH. This may cause erratic behavior,
  //in this case toggling the laser on/off multiple times. To combat this, the sketch
  //will 'debounce' the button by reading multiple times over a period of time. If a 
  //button is read as 'high' for multiple successive reads, then the signal was an
  //actual button event.
  //See http://arduino.cc/en/Tutorial/Debounce
  
  int reading = digitalRead(PUSHBUTTON_PIN);  //read from the digital pin PUSHBUTTON - keep in mind that a HIGH reading might be a false reading, so it must be filtered through the debounce code

  //check if the current digitalRead() is different from the previous button state
  if (reading != lastButtonState) 
  {
    lastDebounceTime = millis(); //a change was detected, so reset the debouncing timer by setting it to the current time
  } 

  //check if the amount of time that has passed between now and the lastDebounceTime is more than the
  //debounceDelay
  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    //check if the button state has changed:
    if (reading != buttonState) 
    {
      buttonState = reading;  //the last digitalRead was correct, so set the butonState to the value of reading
      
      //check if the button's current state is HIGH(which signals the program to toggle the laser)
      if (buttonState == HIGH) 
      {
        triggerServo.write(FIRE_POSITION); // move the servo to FIRE_POSITION to fire the dart gun
        delay(300);                        //wait for 300ms
        triggerServo.write(90);            // sets the servo position to 90 degress, centered
        delay(300);    
      }
    }
  }
  lastButtonState = reading;  //set the lastButtonState to the value of reading for the next loop


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

