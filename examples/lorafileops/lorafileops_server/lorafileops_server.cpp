// lorafileops_server.cpp
// -*- mode: C++ -*-
//
// Example program demonstrating how to use
// RH_LoRaFileOps driver on RPi+Linux with a radio supported by
// the LoRa-file-ops Linix driver, such as the SX1278.
// See See https://github.com/starnight/LoRa/tree/file-ops
//
// You can build this in the top level RadioHead directory with something like:
// cd RadioHead
// g++ -o lorafileops_server -I . RH_LoRaFileOps.cpp RHGenericDriver.cpp tools/simMain.cpp examples/lorafileops/lorafileops_server/lorafileops_server.cpp
//
// And run with
// sudo ./lorafileops_server 
//  (root needed to open /dev/loraSPI0.0
//
// Ensure a RadioHead LoRa compatible transmitter is running
// with modem config RH_RF95::Bw125Cr45Sf2048
// eg examples/rf95/rf95_client/rf95_client.pde 
// or
// examples/lorafileops/lorafileops_client/lorafileops_client.cpp

#include <RH_LoRaFileOps.h>

// An instance of the RadioHead LoraFileOps driver
RH_LoRaFileOps lora("/dev/loraSPI0.0");

void setup()
{
  if (!lora.init())
    Serial.println("init failed");
  // Defaults after init are:
  // Centre frequency 434.0 MHz
  // 13dBm transmit power
  // Bandwidth 125kHz
  // Spreading Factor 2045
  // CRC on

  // But you can change them all:
  //lora.setFrequency(434000000);
  lora.setTxPower(17);
  //lora.setSpreadingFactor(1024);
  //lora.setLNA(10);
  //lora.setBW(250000);
  
}

void loop()
{
  lora.waitAvailable(100);
  
  // Should be a message for us now   
  uint8_t buf[RH_LORAFILEOPS_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  if (lora.recv(buf, &len))
    {
      RH_LoRaFileOps::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(lora.lastRssi(), DEC);
      
      // Send a reply
      uint8_t data[] = "And hello back to you";
      lora.send(data, sizeof(data)); // Returns when packet fully transmitted
      Serial.println("Sent a reply");
    }
  else
    {
      Serial.println("recv failed");
    }
  
}
