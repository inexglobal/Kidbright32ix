#include <KB.h>
KB kb;
int lastSec = 0;
bool colon = true;
void setup() {
  kb.begin();
  
  //kb.setDate(21,7,20);
  //kb.setTime(10,50,20);
  
  //kb.syncTime("INEX","027477001");
}
void loop() {
  int sec = kb.getSecond();
  int hour = kb.getHour();
  int min = kb.getMinute();
  if (lastSec != sec){
    colon = !colon;
    kb.printTime(hour, min, colon);
    lastSec = sec;
   }
}