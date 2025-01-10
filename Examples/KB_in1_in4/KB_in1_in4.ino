#include <KB.h>	
KB kb;
void setup() 
{
  kb.begin();
}
void loop() 
{
 if(kb.in1()==0)
 {
    kb.beep();
    delay(200);  
 }
}
