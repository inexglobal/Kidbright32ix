#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
  kb.out1(1);
  delay(500);
  kb.out1(0);
  delay(500);
}
