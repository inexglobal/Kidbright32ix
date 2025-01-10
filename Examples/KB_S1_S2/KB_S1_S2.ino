#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
  if(kb.S1()==0)
  {
    kb.print("S1");
    kb.beep();
    delay(200);
    kb.clearDisplay();
  }
  if(kb.S2()==0)
  {
    kb.print("S2");
    kb.beep();
    delay(200);
    kb.clearDisplay();
  }
}
