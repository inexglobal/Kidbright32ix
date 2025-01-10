#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
  kb.print(kb.getSecond());
}
