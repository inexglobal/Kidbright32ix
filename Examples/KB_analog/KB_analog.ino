#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
 kb.scroll(kb.analog(2));
}
