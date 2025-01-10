#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
  kb.setDate(21,7,20);
}
void loop() 
{
  kb.scrollDate();
}
