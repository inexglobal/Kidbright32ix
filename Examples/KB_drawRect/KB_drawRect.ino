#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.drawRect(2,2,8,4,1);
  delay(3000);
  kb.drawRect(2,2,8,4,0);
}
void loop() 
{
}
