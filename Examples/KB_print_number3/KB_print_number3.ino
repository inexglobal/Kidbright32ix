#include <KB.h>	
KB kb;
float myFloat = 27.4578;
void setup() 
{
  kb.begin();
  kb.print(myFloat);
}
void loop() 
{
}
