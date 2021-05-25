#include <Encoder.h>

#define ENCAF 2 
#define ENCAR A1  
#define ENCBF 3 
#define ENCBR 49 
#define ENCCF 18 
#define ENCCR 31 
#define ENCDF 19 
#define ENCDR 38

Encoder front_left(ENCAF, ENCAR), front_right(ENCBF, ENCBR), back_left(ENCCF, ENCCR), back_right(ENCDF, ENCDR);

void setup() 
{
  Serial.begin(9600);  
}

void loop() 
{
  Serial.print("front_left: ");
  Serial.println(front_left.read());
  Serial.print("front_right: ");
  Serial.println(front_right.read());
  Serial.print("back_left: ");
  Serial.println(back_left.read());
  Serial.print("back_right: ");
  Serial.println(back_right.read());
  delay(400);
}
