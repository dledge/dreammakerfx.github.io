/***************************************************************
 * Firmware updater for version 1.1.0 DSP firmware
 **************************************************************/


#define FIRMWARE_STR  "v1.1.0"
uint8_t firmware[] = {
    #include "framework-v1.1.0.h"
};



/**************************************************************/
/**************************************************************/


char firmware_rev[] = FIRMWARE_STR;

#define PIN_USR_PB                    (4)

/**
 * Use a push button to trigger the flashing so we don't end up in a flashing
 * loop if someone hits reset.  
 */
bool start_update = false;
void user_pb_pressed(void) {
  start_update = true;
}

void setup() {

  Serial.begin( 115200 );
  int now = millis();
  bool timeout = false;
  while ( !Serial && !timeout) {
    if (millis() > now + 3000) {
      timeout = true;
    }
  }
  
  Serial.println("AFX DSP Firmware Updater ");
  Serial.print("Firmware version: ");
  Serial.println(firmware_rev);
  Serial.println("Press the User push button on the PCB to begin firmware update...");

  // Attach interrupt to the PB
  pinMode(PIN_USR_PB, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_USR_PB), user_pb_pressed, FALLING);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (start_update) {
    Serial.println();
    Serial.println("DSP firmware update started");
    spi_flash_program(firmware, sizeof(firmware));
    start_update = false;
  }

}
