/***********************************************************************************
 *     ______            RobotGeek Desktop RoboTurret v3         ______
 *      |  |                     Gamepad Control                  |  |
 *      |__|_                                                    _|__|
 *   ___|____|_                                                 _|___|___
 *    |       |    _                                        _   |       |
 *   _|_______|____|_                                      _|___|_______|_
 *
 *  The following sketch will allow you to control a Desktop RobotTurret v3 using
 *  the IR Gamepad.
 *
 *  Wiring:
 *    Pan Servo - Digital Pin 10
 *    Tilt Servo - Digital Pin 11
 *    Laser - Digital Pin 2
 *    IR Receiver - Digial Pin 3
 *
 *  Control Behavior:
 *    Up and Down control tilt angle
 *    Left and Right control pan angle
 *    Select toggles Laser output
 *    Start re-centers both pan and tilt servos
 *    B and TA (left and right buttons) control pan movement rate
 *    A and TB (botton and top buttons) control tilt movement rate
 ***********************************************************************************/

//Includes
#include <IRGamepad.h>      //include the IRGamepad library for communication with the gamepad
#include <Servo.h>          //include the servo library for working with servo objects

//Defines
#define BAUDRATE 19200      //define baudrate for optional serial debugging output
#define PAN 10              //Pan Servo Digital Pin
#define TILT 11             //Tilt Servo Digital Pin
#define LASER 2             //Laser Digital Pin
#define IR_INPUT_PIN 3      //IR Receiver Pin

//Constants
const bool useModeB = false;  //Set to true if your controller is switched to mode B
const int maxRate = 20; //Increasing will allow faster speed selection via gamepad

//Objects
Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos
IR_Gamepad myGamepad(IR_INPUT_PIN, useModeB);

//State Variables
int panValue = 90;    //current positional value being sent to the pan servo.
int tiltValue = 90;   //current positional value being sent to the tilt servo.
int laserState = LOW; //The current state of the laser module
int tiltRate = 5; //current tilt movement rate
int panRate = 5; //current pan movement rate

void setup()
{
  Serial.begin(BAUDRATE); //start serial communication at BAUDRATE

  myGamepad.enable(); //Enable the gamepad

  //initialize servos
  panServo.attach(PAN);  // attaches/activates the pan servo on pin PAN
  tiltServo.attach(TILT);  // attaches/activates the tilt servo on pin TILT

  //initalize digital pins
  pinMode(LASER, OUTPUT);      //set the LASER Pin to an output

  //write initial servo positions to set the servos to 'home'
  panServo.write(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.write(tiltValue);//sets the tilt servo position to the default 'home' value

  Serial.print("Desktop RoboTurret v3 Ready\n");    // Send message to serial port
  delay(1000);
}

void loop()
{
  if ( myGamepad.update_button_states() )
  {
    if ( myGamepad.button_press_up() )
    {
      Serial.print( "UP" );
      if ( tiltValue < 180 ) tiltValue += tiltRate;
    }
    if ( myGamepad.button_press_down() )
    {
      Serial.print( "DOWN" );
      if ( tiltValue > 0 ) tiltValue -= tiltRate;
    }
    if ( myGamepad.button_press_left() )
    {
      Serial.print( "LEFT" );
      if ( panValue < 180 ) panValue += panRate;
    }
    if ( myGamepad.button_press_right() )
    {
      Serial.print( "RIGHT" );
      if ( panValue > 0 ) panValue -= panRate;
    }

    if ( myGamepad.button_press_start() )
    {
      Serial.print( "START" );
      tiltValue = 90;
      panValue = 90;
      delay(100);
      myGamepad.update_button_states();
    }
    if ( myGamepad.button_press_select() )
    {
      Serial.print( "SELECT" );
      laserState = !laserState; //Toggle laser state
      digitalWrite( LASER, laserState );
      delay(100);
      myGamepad.update_button_states();
    }

    if ( myGamepad.button_press_b() )
    {
      Serial.print( "B" );
      if ( panRate > 1 ) panRate -= 1;
      delay(100);
      myGamepad.update_button_states();
    }
    if ( myGamepad.button_press_tb() )
    {
      Serial.print( "TB" );
      if ( tiltRate < maxRate ) tiltRate += 1;
      delay(100);
      myGamepad.update_button_states();
    }
    if ( myGamepad.button_press_a() )
    {
      Serial.print( "A" );
      if ( tiltRate > 1 ) tiltRate -= 1;
      delay(100);
      myGamepad.update_button_states();
    }
    if ( myGamepad.button_press_ta() )
    {
      Serial.print( "TA" );
      if ( panRate < maxRate ) panRate += 1;
      delay(100);
      myGamepad.update_button_states();
    }

    Serial.println( " button" );
  }

  panServo.write(panValue);                    // sets the servo position according to the scaled value
  tiltServo.write(tiltValue);                  // sets the servo position according to the scaled value
  delay(15);                                   // waits for the servo to get to the position before going to the next loop
}
