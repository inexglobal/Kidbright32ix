#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
 kb.tone(1200,200);
 delay(500);
}
