#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
 kb.tone(1200);
 delay(200);
 kb.noTone();
 delay(2000);
}
