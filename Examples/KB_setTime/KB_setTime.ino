#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.setTime(10,50,20);
}
void loop() 
{
  kb.scrollTime();
}
