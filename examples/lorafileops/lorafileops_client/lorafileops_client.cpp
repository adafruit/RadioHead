// lorafileops_client.cpp
// -*- mode: C++ -*-
//
// Example program demonstrating how to use
// RH_LoRaFileOps driver on RPi+Linux with a radio supported by
// the LoRa-file-ops Linix driver, such as the SX1278.
// See See https://github.com/starnight/LoRa/tree/file-ops
//
// You can build this in the top level RadioHead directory with something like:
// cd RadioHead
// g++ -o lorafileops_client -I . RH_LoRaFileOps.cpp RHGenericDriver.cpp tools/simMain.cpp examples/lorafileops/lorafileops_client/lorafileops_client.cpp
//
// And run with
// sudo ./lorafileops_client 
//  (root needed to open /dev/loraSPI0.0
//
// Ensure a RadioHead LoRa compatible receiver is running
// with modem config RH_RF95::Bw125Cr45Sf2048
// eg examples/rf95/rf95_server/rf95_server.pde 
// or
// examples/lorafileops/lorafileops_server/lorafileops_server.cpp

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
  Serial.println("Sending to server");
  uint8_t data[] = "Hello World!";
  lora.send(data, sizeof(data));
  lora.waitPacketSent();
  
  // Now wait for a reply
  uint8_t buf[RH_LORAFILEOPS_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (lora.waitAvailableTimeout(6000, 100))
    {
      // Should be a reply message for us now   
      if (lora.recv(buf, &len))
	{
	  Serial.print("got reply: ");
	  Serial.println((char*)buf);
	  Serial.print("RSSI: ");
	  Serial.println(lora.lastRssi(), DEC);    
	  Serial.print("SNR: ");
	  Serial.println(lora.lastSNR(), DEC);    
	}
      else
	{
	  Serial.println("recv failed");
	}
    }
    else
  {
    Serial.println("No reply, is server running?");
  }
  delay(1000);
}
