/***********************************************************************************
 *     ______            RobotGeek Desktop RoboTurret v3         ______
 *      |  |                     Nunchuk Control                  |  | 
 *      |__|_                                                    _|__|
 *   ___|____|_                                                 _|___|___
 *    |       |    _                                        _   |       |
 *   _|_______|____|_                                      _|___|_______|_
 * 
 *  The following sketch will allow you to control a Desktop RobotTurret v3 using
 *  a Wii Nunchuk via the RobotGeek Wii breakout board.
 *    
 *  Wiring
 *    Pan Servo - Digital Pin 10 
 *    Tilt Servo - Digital Pin 11 
 *    Laser - Digital Pin 2
 *    Wii Nunchuk board - IIC Port
 *
 *  Control Behavior:
 *    When not holding Z-Button
 *      The Horizontal Joystick will move the Pan Servo
 *      The Vertical Joystick will move the Tilt Servo
 *    When holding the Z-Button
 *      'Rolling' the nunchuck will move the Pan Servo
 *      'Tilting' the nunchuck will move the Tilt Servo
 *
 *    The C-Button  will toggle the laser module on/off
 *
 *  External Resources
 *
 ***********************************************************************************/
//Includes
#include <Wire.h>            //include the wire library for IIC communication with the nunchuk
#include <ArduinoNunchuk.h>  //include the arduinonunchuk library for parsing the nunchuk data
#include <Servo.h>           //include the servo library for working with servo objects

//Defines
#define BAUDRATE 19200      //define baudrate for optional serial debugging output
#define PAN 10              //Pan Servo Digital Pin
#define TILT 11             //Tilt Servo Digital Pin
#define LASER 2             //Laser Digital Pin
#define JOY_DEADZONE 10     //defines deadband of joystick
#define ACC_DEADZONE 10     //defines deadband of accelerometer
#define JOY_SEN 20          //defines sensitivity factor of joystick. Lower value = more sensitive, higher value = less sensitive
#define ACC_SEN 50          //defines sensitivity factor of accelerometer. Lower value = more sensitive, higher value = less sensitive
#define NUMACCELREADINGS 10 //determines the number of values to average. Increasing this will increase smoothing also increase latency.
#define NUMJOYREADINGS 5    //determines the number of values to average. Increasing this will increase smoothing also increase latency.

Servo panServo, tiltServo;  // create servo objects to control the pan and tilt servos

ArduinoNunchuk nunchuk = ArduinoNunchuk(); // Instantiate Arduino Nunchuk library

// Nunchuk sensor variables
int accx,accy;    //accelerometer X and Y values
byte zbut,cbut;   //z-button and c-button values
int joyx,joyy;    //vertical/horizontal joystick values

// Filtering code variables
int joyx_readings[NUMJOYREADINGS];    // the readings from the joystick X input, array that is NUMJOYREADINGS in length
byte joyx_index = 0;                  // the index of the current reading
int joyx_total = 0;                   // the running total
int joyx_average = 0;                 // the average

int joyy_readings[NUMJOYREADINGS];    // the readings from the joystick Y input, array that is NUMJOYREADINGS in length
byte joyy_index = 0;                  // the index of the current reading
int joyy_total = 0;                   // the running total
int joyy_average = 0;                 // the average

int accx_readings[NUMACCELREADINGS];  // the readings from the accelerometer X input, array that is NUMACCELREADINGS in length
byte accx_index = 0;                  // the index of the current reading
int accx_total = 0;                   // the running total
int accx_average = 0;                 // the average

int accy_readings[NUMACCELREADINGS];  // the readings from the accelerometer Y input, array that is NUMACCELREADINGS in length
byte accy_index = 0;                  // the index of the current reading
int accy_total = 0;                   // the running total
int accy_average = 0;                 // the average

int panValue = 90;   //current positional value being sent to the pan servo. 
int tiltValue = 90;  //current positional value being sent to the tilt servo. 
int pan_temp;        //temporary pan value used for calculations
int tilt_temp;       //temporary tilt value used for calculations

//State Variables
int laserState = LOW;         //The current state of the laser module
int previousCbutState = 0;    // used for on/off state code for the C-Button. Z-Button state is not required

//Timing variables
long lastDebounceTime = 0;  // the last time the output pin was toggled. This variable is a 'long' because it may need to hold many milliseconds, and a 'long' will afford more space than an 'int'
int debounceDelay = 50;    // the amount of time that that a button must be held, for a reading to register (in milliseconds)

void setup() 
{ 
  Serial.begin(BAUDRATE); //start serial communication at BAUDRATE
  
   nunchuk.init();// Initialize Nunchuk
  
  //initialize servos
  panServo.attach(PAN);  // attaches/activates the pan servo on pin PAN 
  tiltServo.attach(TILT);  // attaches/activates the tilt servo on pin TILT 

  //initalize digital pins
  pinMode(LASER, OUTPUT);      //set the LASER Pin to an output
  
  //write initial servo positions to set the servos to 'home'
  panServo.write(panValue);  //sets the pan servo position to the default 'home' value
  tiltServo.write(tiltValue);//sets the tilt servo position to the default 'home' value
  
  //loop through each array and set everything in the filter code to 0
  for (int accx_Reading = 0; accx_Reading < NUMACCELREADINGS; accx_Reading++)
    accx_readings[accx_Reading] = 0;  
  for (int accy_Reading = 0; accy_Reading < NUMACCELREADINGS; accy_Reading++)
    accy_readings[accy_Reading] = 0; 
  for (int joyx_Reading = 0; joyx_Reading < NUMJOYREADINGS; joyx_Reading++)
    joyx_readings[joyx_Reading] = 0;  
  for (int joyy_Reading = 0; joyy_Reading < NUMJOYREADINGS; joyy_Reading++)
    joyy_readings[joyy_Reading] = 0; 
  
  Serial.print("Desktop RoboTurret v3 Ready\n");    // Send message to serial port
  delay(1000);  
} 

void loop() 
{ 
  GetNunchukValues();  //Update Nunchuk global variables from the nunchuk


  /**************  Laser Toggle on/off code *******************************/
  //check if the c-button is pressed (1/HIGH) and that the last state was unpressed (0/LOW)
  if (cbut == 1 && previousCbutState == 0)
  {
    laserState = !laserState; //set the laserState to the opposite of what it was before
  }
  
  digitalWrite(LASER, laserState); //set the pin LASER to the value of laserState
  previousCbutState = cbut;        //set the previous button state for the next loop


  /**************  Pan/Tilt servo movement *******************************/  
    
      /**************  default joystick/pan-tilt behavior ***************/
  //check that the z-button is not being held down, which signifies that the joystick will control the servos    
  if (zbut == 0)
  {
     //GetNunchukValues() will set joyx and joyy to values between -128 and 127 depending on where the joystick is centered.
     //ideally 0 would represent the center of the joystick, where we would expect no movement. However the joystick might not
     //be perfectly centered. The JOY_DEADZONE defines a zone of values where the servos should ignore any input.
     //i.e. for a deadzone of 20, nothing will happen until the value drops below -20 or goes above 20
    
    if(joyx > JOY_DEADZONE || joyx < -JOY_DEADZONE){        // If joystick goes beyond deazone centerpoint...
      pan_temp += joyx/JOY_SEN;                             // Increment temporary variable based upon how far away from center we are
    } 
    if(joyy > JOY_DEADZONE || joyy < -JOY_DEADZONE){
      tilt_temp += joyy/JOY_SEN;    
    }     
  }

      /**************  accelerometer/pan-tilt behavior ***************/
  // when zbutton is held down, accelerometer is used to control pan/tilt
  else if (zbut == 1)
  {
     //GetNunchukValues() will set accx and accy to values between ~ -215 and 215 depending on where the nunhcuk is centered.
     //ideally 0 would represent the the nunchuck being perfectly upright. To give the user some room to have the turret stop
     //the program defines a deadzone (much like the joystick)
     //i.e. for a deadzone of 50, nothing will happen until the value drops below -50 or goes above 50
    if(accx > ACC_DEADZONE || accx < -ACC_DEADZONE){      // If accelerometer goes beyond deazone centerpoint...
      pan_temp += accx/ACC_SEN;                           // Increment temporary variable based upon how far away from center we are
    } 
    if(accy > 10 || accy < -10){
      tilt_temp -= accy/ACC_SEN;     
    }  
  }


  //keep the pan values in easily manageable range
  pan_temp = max(pan_temp, -128);//use the max() function to make sure the value never falls below -128
  pan_temp = min(pan_temp, 128);//use the min() function to make sute the value never goes above 128
  
  //keep the tilt values in easily manageable range
  tilt_temp = max(tilt_temp, -128);//use the max() function to make sure the value never falls below -128
  tilt_temp = min(tilt_temp, 128);//use the min() function to make sute the value never goes above 128
  
  panValue = map(pan_temp, -128, 128, 0, 180);     // scale the pan value to use it with the servo (value between 0 and 180) 
  tiltValue = map(tilt_temp, -128, 128, 0, 180);     // scale the tilt value to use it with the servo (value between 0 and 180
  
  panServo.write(panValue);                    // sets the servo position according to the scaled value 
  tiltServo.write(tiltValue);                  // sets the servo position according to the scaled value 
  delay(15);                                   // waits for the servo to get to the position before going to the next loop

  //  SerialNunchuk(); //Uncomment for data debugging
} 



//sets the global nunchuk variables based on the nunchuk values
void GetNunchukValues()
{
  nunchuk.update();                 //update from Nunchuk library
  zbut = nunchuk.zButton;           //set the zbut global variable based on the nunchuk value
  cbut = nunchuk.cButton;           //set the cbut global variable based on the nunchuk value

  // We recommend using Accel_Filter at all times, it helps tame the very noisy accelerometer data.
  // We recommend using Joystick_Filter for Direct Control sketch, but not for Incremental Control Sketch.

  //Accel();                        //Updates unfiltered values from Joystick. Comment out Accel_Filter(); if you are using this.
  Accel_Filter();                 //Filters and updates values from Accelerometer. Comment out Accel(); if you are using this.

  Joystick();                     //Updates unfiltered values from Accelerometer. Comment out Joystick_Filter(); if you are using this.
  //Joystick_Filter();              //Filters and updates values from Joystick.  Comment out Joystick(); if you are using this.
}

void Joystick()    // Unfiltered data. Makes 0 the centerpoint. Range from approx -128 to 128
{
  joyx = nunchuk.analogX - 128; 
  joyy = nunchuk.analogY - 128;
}

void Accel()      // Unfiltered data. Makes 0 the centerpoint. Range from approx -215 to 215
{
  accx = nunchuk.accelX - 512;  
  accy = nunchuk.accelY - 512;
}

void Joystick_Filter()      //Filtered Joystick Data & Updates
{
  // smoothing code
  // subtract the last reading:
  joyx_total= joyx_total - joyx_readings[joyx_index];
  joyy_total= joyy_total - joyy_readings[joyy_index];
  // read from the sensor:  
  joyx_readings[joyx_index] = nunchuk.analogX;
  joyy_readings[joyy_index] = nunchuk.analogY;
  // add the reading to the total:
  joyx_total = joyx_total + joyx_readings[joyx_index];
  joyy_total = joyy_total + joyy_readings[joyy_index];  
  // advance to the next position in the array:  
  joyx_index = joyx_index + 1;    
  joyy_index = joyy_index + 1;   
  // if we're at the end of the array...
  if (joyx_index >= NUMJOYREADINGS)
    // ...wrap around to the beginning: 
    joyx_index = 0;               
  // if we're at the end of the array...  
  if (joyy_index >= NUMJOYREADINGS)  
    // ...wrap around to the beginning:   
    joyy_index = 0;  
  // calculate the average:
  joyx_average = joyx_total / NUMJOYREADINGS;  
  joyy_average = joyy_total / NUMJOYREADINGS;  
  //write average to accelerometer value variable  
  joyx = joyx_average - 128;  // makes 0 the centerpoint. Range from approx -128 to 128
  joyy = joyy_average - 128;
}

void Accel_Filter()      //Filtered Accelerometer Data & Updates
{
  // smoothing code
  // subtract the last reading:
  accx_total= accx_total - accx_readings[accx_index];
  accy_total= accy_total - accy_readings[accy_index];
  // read from the sensor:  
  accx_readings[accx_index] = nunchuk.accelX;
  accy_readings[accy_index] = nunchuk.accelY;
  // add the reading to the total:
  accx_total = accx_total + accx_readings[accx_index];
  accy_total = accy_total + accy_readings[accy_index];  
  // advance to the next position in the array:  
  accx_index = accx_index + 1;    
  accy_index = accy_index + 1;   
  // if we're at the end of the array...
  if (accx_index >= NUMACCELREADINGS)
    // ...wrap around to the beginning: 
    accx_index = 0;               
  // if we're at the end of the array...  
  if (accy_index >= NUMACCELREADINGS)  
    // ...wrap around to the beginning:   
    accy_index = 0;  
  // calculate the average:
  accx_average = accx_total / NUMACCELREADINGS;  
  accy_average = accy_total / NUMACCELREADINGS;  
  //write average to accelerometer value variable  
  accx = accx_average - 512;  // makes 0 the centerpoint. Range from approx -215 to 215
  accy = accy_average - 512;
}  

void SerialNunchuk()      // Serial output of raw Nunchuk sensor values for debugging
{
  Serial.print(nunchuk.analogX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.analogY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelX, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelY, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.accelZ, DEC);
  Serial.print(' ');
  Serial.print(nunchuk.zButton, DEC);
  Serial.print(' ');
  Serial.println(nunchuk.cButton, DEC);
  delay(10);
}


