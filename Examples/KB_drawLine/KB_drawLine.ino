#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.drawLine(1,1,14,6,1);
  delay(3000);
  kb.drawLine(1,1,14,6,0);
}
void loop() 
{
}
