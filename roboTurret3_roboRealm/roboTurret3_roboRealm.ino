/**********************************
 * This code is based off the original 
 *	RoboRealm_Arduino.pde
 * by Sparkfun. This sketch will turn the Arduino into
 * a passthrough device, allowing it to be controlled by
 * the computer. 
 * 
 * Reference:
 * http://www.roborealm.com/help/Sparkfun_Arduino.php
 **********************************/


#include <Servo.h> 

Servo servos[12];
boolean pinModes[14];

unsigned int crc;
unsigned int command;
unsigned int channel;
unsigned int value;
unsigned int valueLow;
unsigned int valueHigh;
unsigned int streamDigital;
unsigned int streamAnalog;
unsigned int lastDigital;
unsigned int lastAnalog[8];
unsigned int heartBeat=0;

unsigned int defaultServo[14];


#define ARDUINO_GET_ID 0
#define ARDUINO_SET_SERVO 1
#define ARDUINO_SET_DIGITAL_STREAM 2
#define ARDUINO_SET_DIGITAL_HIGH 3
#define ARDUINO_SET_DIGITAL_LOW 4
#define ARDUINO_SET_ANALOG_STREAM 5
#define ARDUINO_DIGITAL_STREAM 6
#define ARDUINO_ANALOG_STREAM 7
#define ARDUINO_SET_ANALOG 8

#define ARDUINO_SET_SERVO_DEFAULT 9

#define ARDUINO_HEARTBEAT 10


void initialize()
{
  int i;
  for (i=2;i<14;i++) pinModes[i]=-1;
  streamDigital=0;
  streamAnalog=0;
  lastDigital=-1;
  for (i=0;i<8;i++) lastAnalog[i]=-1;

  for (i=0;i<12;i++) defaultServo[i]=1500;

}

void setup() 
{
  Serial.begin(115200);

  initialize();
}

void writePacket()
{
  unsigned char buffer[2];  
  buffer[0]=command|128;
  buffer[1]=channel;
  Serial.write(buffer, 2);
}

void writeValuePacket(int value)
{
  unsigned char buffer[5];  
  
  buffer[0]=command|128;
  buffer[1]=channel;
  buffer[2]=value&127;
  buffer[3]=(value>>7)&127;
  buffer[4]=(buffer[0]^buffer[1]^buffer[2]^buffer[3])&127;
  
  Serial.write(buffer, 5);
}

void readPacket()
{
  // get header byte
  // 128 (bit 8) flag indicates a new command packet .. that 
  // means the value bytes can never have 128 set!
  // next byte is the command 0-8
  // next byte is the channel 0-16

  do
  {
    while (Serial.available() <= 0) continue; 
    command = Serial.read();
  }
  while ((command&128)==0);
  
  command^=128;

  while (Serial.available() <= 0) continue; 
  channel = Serial.read();
}

int readValuePacket()
{
  unsigned int valueLow;
  unsigned int valueHigh;
  
  // wait for value low byte    
  while (Serial.available() <= 0) continue; 
  valueLow = Serial.read();
  if (valueLow&128) return 0;

  // wait for value high byte    
  while (Serial.available() <= 0) continue; 
  valueHigh = Serial.read();
  if (valueHigh&128) return 0;
    
  // wait for crc byte    
  while (Serial.available() <= 0) continue; 
  crc = Serial.read();
  if (crc&128) return 0;
  
  if (crc!=(((128|command)^channel^valueLow^valueHigh)&127)) return 0;

  value = valueLow|(valueHigh<<7);
  
  return 1;
}

void loop() 
{
  while (Serial.available()>0)
  {
    readPacket();
    
    heartBeat=0;



    switch (command)
    {
      // init
      case  ARDUINO_GET_ID:
        initialize();
        Serial.print("ARDU");
      break;
      // servo
      case  ARDUINO_SET_SERVO:
        if ((channel>=3)&&(channel<=11))
        {
          if (readValuePacket())
          {
            if (pinModes[channel]!=1)
            {
              servos[channel].attach(channel);
              pinModes[channel]=1;
            }
            servos[channel].writeMicroseconds(value);
            writeValuePacket(value);
          }
        }
      break;
      //digital stream
      case  ARDUINO_SET_DIGITAL_STREAM:
        if (readValuePacket())
        {
          streamDigital = value;
          writeValuePacket(value);
          lastDigital=-1;
        }
      break;
      //set digital high
      case  ARDUINO_SET_DIGITAL_HIGH:
        if ((channel>=2)&&(channel<14))
        {
          if (pinModes[channel]!=2)
          {
            if (pinModes[channel]==1)
              servos[channel].detach();

            pinMode(channel, OUTPUT);
            pinModes[channel]=2;
            if (streamDigital&(1<<channel))
              streamDigital^=1<<channel;
          }
          
          digitalWrite(channel, HIGH);
          writePacket();
        }
      break;
      //set digital low
      case  ARDUINO_SET_DIGITAL_LOW:
        if ((channel>=2)&&(channel<14))
        {
          if (pinModes[channel]!=2)
          {
            if (pinModes[channel]==1)
              servos[channel].detach();
              
            pinMode(channel, OUTPUT);
            pinModes[channel]=2;
            
            if (streamDigital&(1<<channel))
              streamDigital^=1<<channel;
          }
          digitalWrite(channel, LOW);
          writePacket();
        }
      break;
      //analog stream
      case  ARDUINO_SET_ANALOG_STREAM:
        if (readValuePacket())
        {
          streamAnalog = value;
          writeValuePacket(value);
          for (channel=0;channel<8;channel++) lastAnalog[channel]=-1;
        }
      break;
      case  ARDUINO_SET_ANALOG:
        if (readValuePacket())
        {
					if ((channel>=3)&&(channel<=11))
					{
						if (pinModes[channel]!=2)
						{
							if (pinModes[channel]==1)
								servos[channel].detach();

							pinMode(channel, OUTPUT);
							pinModes[channel]=2;
						}
						analogWrite(channel, value);
						writeValuePacket(value);
					}
        }
      break;
      case  ARDUINO_SET_SERVO_DEFAULT:

        if ((channel>=3)&&(channel<=11))

        {

          if (readValuePacket())

          {

            defaultServo[channel] = value;

						writeValuePacket(value);

          }

        }

      break;

      case ARDUINO_HEARTBEAT:

      break;

    }
  }
  
  if ((heartBeat++)>25000)

  {

    int i;

    for (i=3;i<12;i++)

    {

      if (pinModes[i]==1) 

        servos[i].writeMicroseconds(defaultServo[i]);

      else

      if (pinModes[i]==2) 

        analogWrite(i, 0);

    }

  }



  for (channel=0;channel<8;channel++)
  {
    if (streamAnalog&(1<<channel))
    {
      value = analogRead(channel);
      // only send value if it has changed
      if (value!=lastAnalog[channel])
      {
        command = ARDUINO_ANALOG_STREAM;
        writeValuePacket(value);
        
        lastAnalog[channel]=value;
      }
    }
  }

  if (streamDigital)
  {
    value=0;
    for (channel=2;channel<14;channel++)
    {
      if (streamDigital&(1<<channel))
      {
        if (pinModes[channel]!=3)
        {
          if (pinModes[channel]==1)
            servos[channel].detach();
            
          pinMode(channel, INPUT);
          pinModes[channel]=3;
          // pullup
          digitalWrite(channel, HIGH); 
        }
        
        value |= digitalRead(channel)<<channel;
      }
    }

    // only send value if it has changed
    if (value!=lastDigital)
    {    
      command = ARDUINO_DIGITAL_STREAM;
      writeValuePacket(value);

      lastDigital=value;
    }
  }
}


