#include <KB.h>
KB kb;
void setup() {
  kb.begin();
}
void loop() {
  kb.printNumber(kb.light());
  delay(100);
}