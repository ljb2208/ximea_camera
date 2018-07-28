

/**********************************************
 * @file camera_trigger,ino
 * @author Brian Goldfain <bgoldfai@gmail.com>
 * @date January 20, 2017
 * @copyright 2017 Georgia Institute of Technology
 * @brief Triggers the cameras
 *
 ***********************************************/


#include <avr/io.h>
#include <avr/interrupt.h>

int triggerPin = 3; ///< Pin number on the Arduino Micro used to trigger the cameras
int triggerFPS = 40; ///< default trigger rate

unsigned long time; ///< Time that the last message was published
unsigned long elapsed; ///< Calculated elapsed time since last transmission

unsigned long pps; ///< Time that last pps message was recived
int ppsCount=0; ///< count of received pps pulses
float dataPublishPeriod = 500; ///< Period for data to be sent back to computer

/**
* @brief Sets up all parameters and attaches interrupts
*/
void setup()
{
  pinMode(triggerPin, OUTPUT);
  
  //attach all interrupt for incoming pps
  //attachInterrupt(digitalPinToInterrupt(triggerPin), int0, RISING);
  
  Serial.begin(115200);
}

/**
* @brief Main body of program, processes data from USB, sends diagnostic info over USB.
*/
void loop()
{
  //parse incoming fps trigger requests from computer
  if(Serial.available() && Serial.read() == '#')
  {
      char buffer[10];
      int numBytesRead = Serial.readBytesUntil(':', buffer, 10);
      if(numBytesRead < 10)
        buffer[numBytesRead] = '\0';
      if(strcmp(buffer, "trg") == 0)
      {        
        trigger();
      }
  }  
}

void trigger()
{      
  digitalWrite(triggerPin, HIGH);
  //delay(1);
  digitalWrite(triggerPin, LOW);
  //digitalWrite(triggerPin, !digitalRead(triggerPin));  
}


