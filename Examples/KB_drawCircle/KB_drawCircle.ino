#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.fillCircle(4,4,3,1);
  delay(3000);
  kb.fillCircle(4,4,3,0);
}
void loop() 
{
}
