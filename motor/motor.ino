/*

  Project: ADC recorder
  Started: 2 July 2016

*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

#define ledPin 13


// Stepper motor
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *myMotor;

int state = 1;

//
//
void setup()
{
     // Serial output
     Serial.begin( 9600 );           // set up Serial library at 9600 bps

     // ADC
     pinMode( ledPin, OUTPUT );
     
     digitalWrite( ledPin, state );
     
     // Motor
     myMotor = AFMS.getStepper( 200, 2 );
     // myMotor->setSpeed( 60 );
     Serial.println( "Setup configured motor.");

}


//
//
void loop()
{
     digitalWrite( ledPin, HIGH );
     Serial.println( "Stepping motor." );
     
     myMotor->step( 100, FORWARD, SINGLE );

     digitalWrite( ledPin, LOW );
     delay( 1000 );
     
     
}
