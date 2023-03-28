// rf22_cw.ino
// -*- mode: C++ -*-
// Example sketch showing how to emit a continuous carrier wave (CW)
// for test purposes
// Tested on Duemilanove, Uno with Sparkfun RFM22 wireless shieldg

#include <SPI.h>
#include <RH_RF22.h>

// Singleton instance of the radio driver
RH_RF22 rf22;

void setup() 
{
  Serial.begin(9600);
  if (!rf22.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36

  // CW mode:
  rf22.setModemConfig(RH_RF22::UnmodulatedCarrier);

}

void loop()
{
  rf22.setModeTx();
  delay(1000);
  rf22.setModeIdle();
  delay(1000);
}
