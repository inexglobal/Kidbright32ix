#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.syncTime("INEX","027477001");
}
void loop() 
{
  kb.scrollDateTime();
}
