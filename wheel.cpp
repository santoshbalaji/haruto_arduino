#include "wheel.h"

Wheel::Wheel()
{
  pinMode(PWMA, OUTPUT);
  pinMode(MAF, OUTPUT);
  pinMode(MAR, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(MBF, OUTPUT);
  pinMode(MBR, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(MCF, OUTPUT);
  pinMode(MCR, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(MDF, OUTPUT);
  pinMode(MDR, OUTPUT);
}

void Wheel::executeCommand()
{
  analogWrite(PWMA, frontLeftCommandSpeed);
  analogWrite(PWMB, frontRightCommandSpeed);
  analogWrite(PWMC, backLeftCommandSpeed);
  analogWrite(PWMD, backRightCommandSpeed);
  
  if(frontLeftExpectedSpeed > 0)
  {   
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, LOW);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, LOW);
  }
  else if(frontLeftExpectedSpeed < 0)
  {
    digitalWrite(MAF, LOW);
    digitalWrite(MAR, HIGH);
    digitalWrite(MCF, LOW);
    digitalWrite(MCR, HIGH);  
  }
  else
  {
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, HIGH);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, HIGH);    
  }
  
  if(frontRightExpectedSpeed > 0)
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, LOW);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, LOW);
  }
  else if(frontRightExpectedSpeed < 0)
  {
    digitalWrite(MBF, LOW);
    digitalWrite(MBR, HIGH);
    digitalWrite(MDF, LOW);
    digitalWrite(MDR, HIGH);  
  }
  else
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, HIGH);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, HIGH);    
  }
}
