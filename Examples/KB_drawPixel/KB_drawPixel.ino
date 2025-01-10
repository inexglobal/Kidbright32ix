#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.drawPixel(3,3,1);
  delay(3000);
  kb.drawPixel(3,3,0);
}
void loop() 
{
}
