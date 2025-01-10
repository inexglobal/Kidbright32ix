#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.drawTriangle(1,5,8,0,14,7,1);
  delay(3000);
  kb.drawTriangle(1,5,8,0,14,7,0);
}
void loop() 
{
}
