#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.waitS2();
}
void loop() 
{
  kb.ledWIFI(1);
  delay(500);
  kb.ledWIFI(0);
  delay(500);
}
