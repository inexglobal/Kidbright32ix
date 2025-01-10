#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
 kb.beep();
 delay(500);
}
