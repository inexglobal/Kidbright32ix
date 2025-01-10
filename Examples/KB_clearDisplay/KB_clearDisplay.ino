#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
  kb.print("OK!");
  delay(1000);
  kb.clearDisplay();
  delay(1000);
}
