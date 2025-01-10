#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
 }
void loop() 
{
 double temp = kb.temperature();
 kb.scroll(String(temp,0));
}
