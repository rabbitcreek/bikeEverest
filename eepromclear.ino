#include <EEPROM.h>
#define EEPROM_SIZE 64
int address = 1;
void setup() {
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM"); delay(1000000);
  }
  EEPROM.write(0,0);
  EEPROM.commit();

}

void loop() {
  // put your main code here, to run repeatedly:

}
