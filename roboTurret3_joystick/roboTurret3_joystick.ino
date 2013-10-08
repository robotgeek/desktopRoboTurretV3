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
#define PAN 10        //Pan Servo Digital Pin
#define TILT 11       //Tilt Servo Digital Pin
#define LASER 2       //Laser Digital Pin
#define H_JOY 0       //Horizontal Joystick Analog Pin
#define V_JOY 1       //Vertical Joystick Analog Pin 
#define PUSHBUTTON 4  //Pushbutton Digital Pin
#define JOY_SENSITIVITY 100 //defines sensitivity factor of joystick. Lower value = more sensitive, higher value = less sensitive
#define DEADBANDLOW 480
#define DEADBANDHIGH 540

Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos
int horizontalValue, verticalValue;  //variables to hold the last reading from the analog pins for the horizontal and vertical joystick



int panValue = 90;   //current positional value being sent to the pan servo. 
int tiltValue = 90;  //current positional value being sent to the tilt servo. 


//State Variables
int laserState = LOW;         //The current state of the laser module
int buttonState;             // the current state of the pushbuton
int lastButtonState = LOW;   // the previous reading from the input pin

//Timing variables
long lastDebounceTime = 0;  // the last time the output pin was toggled. This variable is a 'long' because it may need to hold many milliseconds, and a 'long' will afford more space than an 'int'
int debounceDelay = 50;    // the amount of time that that a button must be held, for a reading to register (in milliseconds)

void setup() 
{ 
  //initialize servos
  panServo.attach(PAN);  // attaches/activates the pan servo on pin PAN 
  tiltServo.attach(TILT);  // attaches/activates the tilt servo on pin TILT 

  //initalize digital pins
  pinMode(PUSHBUTTON, INPUT);  //set the PUSHBUTTON Pin to an Input
  pinMode(LASER, OUTPUT);      //set the LASER Pin to an output
  
  //Analog pins do not need to be initialized
  
  //write initial servo positions to set the servos to 'home'
  panServo.write(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.write(tiltValue);//sets the tilt servo position to the default 'home' value
  
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
  
  int reading = digitalRead(PUSHBUTTON);  //read from the digital pin PUSHBUTTON - keep in mind that a HIGH reading might be a false reading, so it must be filtered through the debounce code

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
        laserState = !laserState;  //set the laserState to the opposite of what it was set to before
      }
    }
  }

  // set the Laser:
  digitalWrite(LASER, laserState);

  lastButtonState = reading;  //set the lastButtonState to the value of reading for the next loop



  /**************Servo Positions *******************************/
  //read the values from the analog sensors/joysticks
   horizontalValue = analogRead(H_JOY);
   verticalValue = analogRead(V_JOY);
   
   //check that the joystick is outisde of the deadband. Movements in the deadband should not register
   if(horizontalValue < DEADBANDLOW || horizontalValue > DEADBANDHIGH)
   {
     panValue = panValue + (horizontalValue - 512)/JOY_SENSITIVITY;
     panValue = max(panValue, 0);  //use the max() function to make sure the value never falls below 0
     panValue = min(panValue, 180);//use the min() function to make sute the value never goes above 180
   }   
   
   //check that the joystick is outisde of the deadband. Movements in the deadband should not register
   if(verticalValue < DEADBANDLOW || verticalValue > DEADBANDHIGH)
   {
     tiltValue = tiltValue + (verticalValue - 512)/JOY_SENSITIVITY;
     tiltValue = max(tiltValue, 0);//use the max() function to make sure the value never falls below 0
     tiltValue = min(tiltValue, 180);//use the min() function to make sute the value never goes above 180
   }
  panServo.write(panValue);                  // sets the servo position according to the scaled value 
  tiltServo.write(tiltValue);                  // sets the servo position according to the scaled value 
   
  delay(15);                           // waits for the servo to get to they're position 
} 

