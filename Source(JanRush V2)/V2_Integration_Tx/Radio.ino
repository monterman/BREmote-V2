void setRadioActivityEnabled(bool enabled)
{
  radio_activity_enabled = enabled;

  if(!radio_driver_ready) return;

  if(!enabled)
  {
    rfInterrupt = false;
    radio.sleep();
    return;
  }

  radio.setDio2AsRfSwitch(true);
  radio.setCRC(0);
  radio.setRxBandwidth(250);
  radio.implicitHeader(6);
  radio.startReceive();
}

bool isRadioActivityEnabled()
{
  return radio_activity_enabled;
}

void radioErrorHalt(int type)
{
  if(type == 1) while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_P, 200);
  if(type == 2) while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_C, 200);
  while(1) scroll4Digits(LET_E, LET_H, LET_F, LET_I, 200);
}

void radioInitSuccess()
{
  radio_driver_ready = true;
  setRadioActivityEnabled(radio_activity_enabled);
}

void startupRadio()
{
  initRadioHardware();
}

void ICACHE_RAM_ATTR packetReceived(void) 
{
  // we sent or received a packet, set the flag
  rfInterrupt = true;
}

// Function to initiate pairing
bool initiatePairing() 
{
  if(!isRadioActivityEnabled()) return false;

  uint8_t dest_address[3];

  rxprintln("Initiating Pairing...");
  usrConf.paired = false;
  
  // TODO: Implement address conflict detection during pairing
  //if (!checkAndAdjustAddress())
  //{
  //    return false;  // Too many conflicts
  //}
  
  uint8_t pairingPacket[8];  // 0xAB + 3 bytes address + CRC (up to 8 bytes for confirmation)
  unsigned long startTime = millis();
  
  // Prepare pairing packet
  pairingPacket[0] = 0xAB;
  memcpy(pairingPacket + 1, usrConf.own_address, 3);
  pairingPacket[4] = esp_crc8(pairingPacket, 4);
  
  while (millis() - startTime < PAIRING_TIMEOUT) 
  {
    if(!isRadioActivityEnabled()) return false;
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
      if(!isRadioActivityEnabled()) return false;
      while(!rfInterrupt && millis() - responseTime < 1000) delay(10);
      delay(10);
      
      if (rfInterrupt && radio.readData(responseBuffer, 15) == RADIOLIB_ERR_NONE) 
      {
        rfInterrupt = false;
        rxprintln("Received response");
        if (responseBuffer[0] == 0xBA && memcmp(responseBuffer + 1, usrConf.own_address, 3) == 0)
        {
          rxprintln("Address correct");
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
  if(!isRadioActivityEnabled())
  {
    Serial.println("Radio activity is disabled, pairing skipped.");
    return;
  }

  if(!usrConf.paired)
  {
    Serial.println("Not Paired!");
    while(!usrConf.paired && isRadioActivityEnabled())
    {
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
      updateDisplay();
      initiatePairing();
    }
    if(!isRadioActivityEnabled())
    {
      Serial.println("Pairing aborted because radio activity was disabled.");
      return;
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
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
      scroll4Digits(5, LET_A, LET_V, LET_E, 120);
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
    esp_task_wdt_reset();
    if(usrConf.paired && isRadioActivityEnabled())
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
        sendArray[3] = calcFinalThrottle();
        //float steer_mult = ((127.0-(float)steer_scaled) * (float)gear / (float)usrConf.max_gears)+127.0;
        sendArray[4] = steer_scaled;//(uint8_t)steer_mult;
      }

      thr_sent = sendArray[3];
      steer_sent = sendArray[4];

      sendArray[5] = esp_crc8(sendArray, 5);

      rxprint("Sending: ");
      #ifdef DEBUG_RX
      printHexArray(sendArray, 6);
      #endif

      radio.implicitHeader(6);
      radio.startTransmit(sendArray, 6);
      num_sent_packets++;
      vTaskDelay(pdMS_TO_TICKS(10));
      radio.implicitHeader(6);
      rfInterrupt = false;
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
    if(!isRadioActivityEnabled()) continue;
    //wait until interrupt
    while(!rfInterrupt && isRadioActivityEnabled()) vTaskDelay(pdMS_TO_TICKS(5));
    if(!isRadioActivityEnabled()) continue;

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

// getLinkQuality() is now in ../Common/RadioCommon.h
