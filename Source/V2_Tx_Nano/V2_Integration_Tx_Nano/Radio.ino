void startupRadio()
{
  Serial.print("Starting Radio...");

  SPI.begin(P_SPI_SCK, P_SPI_MISO, P_SPI_MOSI);

  if(usrConf.rf_power < -9 || usrConf.rf_power > 22)
  {
    Serial.println("Error, invalid transmit power");
    while(1) displayError(1);
  }

  Serial.print(" Power: ");
  Serial.print(usrConf.rf_power);

  int state;

  if(usrConf.radio_preset == 1)
  {
    Serial.print(" Region: EU868");
    //869.4-869.65MHz, 10%TOA, 500mW
    //Checked allowed in: EU, Switzerland
                        //          5..12 5..8                                  -9..22             >=1
                        //fc     bw    sf cr                                      pwr              pre tcxo  ldo
    state = radio.begin(869.525, 250.0, 6, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, usrConf.rf_power, 8, 1.8, false);
  }
  else if(usrConf.radio_preset == 2)
  {
    //Reserved for US
    Serial.print(" Region: US/AU915");
                        //          5..12 5..8                                  -9..22             >=1
                        //fc     bw    sf cr                                      pwr              pre tcxo  ldo
    state = radio.begin(915.0, 250.0, 6, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, usrConf.rf_power, 8, 1.8, false);
  }
  else if (usrConf.radio_preset == 3)
  {
    //Reserverd for other country
    Serial.println("Error, unsupported HF setting");
    while(1) displayError(1);
  }
  else
  {
    //Invalid
    Serial.println("Error, unsupported HF setting");
    while(1) displayError(1);
  }

  //radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.implicitHeader(4);
  radio.setCRC(0);
  radio.setRxBandwidth(250);

  Serial.print(" TOA: ");
  Serial.print(radio.getTimeOnAir(4));

  if (state == RADIOLIB_ERR_NONE) 
  {
    Serial.println(" Done");
  } 
  else 
  {
    Serial.print(" Failed, code: ");
    Serial.println(state);
    while (true) displayError(1);
  }
}

void ICACHE_RAM_ATTR packetReceived(void) 
{
  // we sent or received a packet, set the flag
  rfInterrupt = true;
}

// Function to initiate pairing
bool initiatePairing() 
{
  uint8_t dest_address[3];

  rxprintln("Initiating Pairing...");
  usrConf.paired = false;
  
  // First check for address conflicts
  if (0)//!checkAndAdjustAddress()) 
  {
      return false;  // Too many conflicts
  }
  
  uint8_t pairingPacket[7];  // 0xAB + 3 bytes address + CRC
  unsigned long startTime = millis();
  
  // Prepare pairing packet
  pairingPacket[0] = 0xAB;
  memcpy(pairingPacket + 1, usrConf.own_address, 3);
  pairingPacket[4] = esp_crc8(pairingPacket, 4);
  
  while (millis() - startTime < PAIRING_TIMEOUT) 
  {
    unsigned long responseTime = millis();
    rxprintln("Sending pairing request packet: ");
    #ifdef DEBUG_RX
    printHexArray(pairingPacket,5);
    #endif
    // Send pairing request
    radio.implicitHeader(5);
    radio.startTransmit(pairingPacket, 5);
    delay(10);
    radio.implicitHeader(8);
    radio.startReceive();
    rfInterrupt = false;
    // Wait for response
    uint8_t responseBuffer[15];
    
    while(millis() - responseTime < 1000)
    {    
      while(!rfInterrupt && millis() - responseTime < 1000) delay(10);
      delay(10);
      
      if (rfInterrupt && radio.readData(responseBuffer, 15) == RADIOLIB_ERR_NONE) 
      {
        rfInterrupt = false;
        rxprintln("Received response");
        if (responseBuffer[0] == 0xBA && memcmp(responseBuffer + 1, usrConf.own_address, 3) == 0)
        {
          rxprintln("Adress correct");
          // Verify CRC of received packet
          uint8_t receivedCRC = responseBuffer[7];
          uint8_t calculatedCRC = esp_crc8(responseBuffer, 7);
          
          if (receivedCRC == calculatedCRC) 
          {
            rxprintln("CRC correct");
            // Save the other device's address
            memcpy(dest_address, responseBuffer + 4, 3);
            
            // Send final confirmation
            pairingPacket[0] = 0xAC;
            memcpy(pairingPacket + 1, dest_address, 3);
            memcpy(pairingPacket + 4, usrConf.own_address, 3);
            
            // Calculate CRC
            pairingPacket[7] = esp_crc8(pairingPacket, 7);
            
            delay(100);
            rxprintln("Sending response: ");
            #ifdef DEBUG_RX
            printHexArray(pairingPacket, 8);
            #endif
            radio.implicitHeader(8);
            for(int i = 0; i < 3; i++)
            {
              radio.startTransmit(pairingPacket, 8);
              delay(300);
            }

            usrConf.dest_address[0] = dest_address[0];
            usrConf.dest_address[1] = dest_address[1];
            usrConf.dest_address[2] = dest_address[2];
            usrConf.paired = true;
            return true;
          }
        }
      }
      else rfInterrupt = false;
    }
  }
  return false;
}

void checkPairing()
{
  if(!usrConf.paired)
  {
    Serial.println("Not Paired!");
    while(!usrConf.paired)
    {/*
      displayDigits(LET_E, LET_P);
      updateDisplay();
      uint8_t pair_animation = 0;
      while(!tog_input)
      {
        pair_animation++;
        if(pair_animation > 10) pair_animation = 0;
        if(pair_animation > 8)
        {
          displayDigits(TLT, TLT);
        }
        else
        {
          displayDigits(LET_E, LET_P);
        }
        updateDisplay();
        delay(100);
      }
      displayDigits(LET_P, LET_A);
      updateDisplay();*/
      initiatePairing();
    }
    Serial.println("Pairing Done.");
    
    Serial.print("Own Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(usrConf.own_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    Serial.print("Destination Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(usrConf.dest_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    if(usrConf.paired == true)
    {
      saveConfToSPIFFS(usrConf);
    }
  }
}


void sendData(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  while(1)
  {
    if(usrConf.paired)
    {
      // Dest1, Dest2, Dest3, THR, Steer, CRC8
      uint8_t sendArray[6];

      memcpy(sendArray, usrConf.dest_address, 3);

      if(system_locked)
      {
        sendArray[3] = 0;
        sendArray[4] = 127;
      }
      else
      {
        float thr_mult = (float)expoThrCurve(thr_scaled) * (float)(gear+1) / (float)usrConf.max_gears;
        sendArray[3] = (uint8_t)thr_mult;
        //float steer_mult = ((127.0-(float)steer_scaled) * (float)gear / (float)usrConf.max_gears)+127.0;
        sendArray[4] = steer_scaled;//(uint8_t)steer_mult;
      }

      sendArray[5] = esp_crc8(sendArray, 5);

      rxprint("Sending: ")
      #ifdef DEBUG_RX
      printHexArray(sendArray, 6);
      #endif

      radio.implicitHeader(6);
      radio.startTransmit(sendArray, 6);
      num_sent_packets++;
      vTaskDelay(pdMS_TO_TICKS(10));
      rfInterrupt = false;
      radio.implicitHeader(6);
      radio.startReceive();
      //trigger waitForTelemetry
      xTaskNotifyGive(triggeredWaitForTelemetryHandle);
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void waitForTelemetry(void *parameter)
{
  while (1) 
  {
    //wait until called
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    //wait until interrupt
    while(!rfInterrupt) vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t rcvArray[6];
    if (radio.readData(rcvArray, 6) == RADIOLIB_ERR_NONE) 
    {
      rfInterrupt = false;
      rxprint("Received packet: ");
      #ifdef DEBUG_RX
      printHexArray(rcvArray,6);
      #endif

      if (memcmp(rcvArray, usrConf.own_address, 3) == 0) 
      {
        rxprintln("Address matches");
        
        if (rcvArray[5] == esp_crc8(rcvArray, 5)) 
        {
          rxprintln("CRC ok");
          num_rcv_packets++;
          
          uint8_t* ptr = (uint8_t*)&telemetry;  
          if (rcvArray[3] < sizeof(TelemetryPacket))
          {
            ptr[rcvArray[3]] = rcvArray[4];
          }
          if(telemetry.error_code)
          {
            remote_error = telemetry.error_code;
          }
          last_packet = millis();
        }
      }
    }
    else
    {
      rxprintln("Rx err");
      rfInterrupt = false;
    }

    local_link_quality = getLinkQuality(radio.getRSSI(), radio.getSNR());

    rxprint("RSSI: ");
    rxprint(radio.getRSSI());
    rxprint(", SNR: ");
    rxprintln(radio.getSNR());
  }
}

int getLinkQuality(float rssi, float snr) {
  // Normalize RSSI: Expected range (-130 dBm to -50 dBm)
  int rssiScore = constrain(map(rssi, -100, -50, 0, 10), 0, 10);

  // Normalize SNR: Typical range (-20 dB to +10 dB)
  int snrScore = constrain(map(snr, -10, 10, 0, 10), 0, 10);

  // Weighted average (adjust weights as needed)
  float combinedScore = (0.7 * rssiScore) + (0.3 * snrScore);

  // Convert to integer and ensure it's in range 0-10
  return constrain(round(combinedScore), 0, 10);
}