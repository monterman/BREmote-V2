// V3 - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz in sendData(); THR capped at 0xF2
// V3 - 2026-04-25 - P7: Added RTM/FM meta-packet queue consumer in sendData(); cap 0xF2→0xF0; queueMetaPacketBurst()
// V3 - 2026-04-29 - Bundle A: radio_preset max clamped to 2; dead foil_speed != 99 sentinel removed
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


// V3 - 2026-04-24 - Added 0xF3 GPS meta-packet burst at 2Hz for Phase B anti-spoofing.
//                   THR capped at 0xF2: 0xF3 is reserved as the GPS meta-packet marker.
void sendData(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  // GPS meta-packet cycle counter. Incremented each control cycle (100ms).
  // Wraps at 5 → resets to 0. The GPS meta-packet fires on the cycle where
  // it resets to 0 (i.e., every 5 × 100ms = 500ms = 2Hz).
  static uint8_t gps_cycle = 0;

  while(1)
  {
    if(usrConf.paired && isRadioActivityEnabled())
    {
      // ---- Meta-packet burst path (highest priority, preempts GPS and control packets) ----
      // Sends one 6-byte meta-packet per iteration until count reaches 0.
      // 3 bursts × 100ms cycle = 300ms total. Type/value written before count by loop task.
      if (rtm_meta_count > 0)
      {
        uint8_t metaPkt[6];
        memcpy(metaPkt, usrConf.dest_address, 3);
        metaPkt[3] = rtm_meta_type;
        metaPkt[4] = rtm_meta_value;
        metaPkt[5] = esp_crc8(metaPkt, 5);

        rxprint("RTM meta-pkt: ");
        #ifdef DEBUG_RX
        printHexArray(metaPkt, 6);
        #endif

        radio.implicitHeader(6);
        radio.startTransmit(metaPkt, 6);
        rtm_meta_count--;
        num_sent_packets++;
        vTaskDelay(pdMS_TO_TICKS(10));
        radio.implicitHeader(6);
        rfInterrupt = false;
        radio.startReceive();
        xTaskNotifyGive(triggeredWaitForTelemetryHandle);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        continue;
      }

      gps_cycle++;
      if (gps_cycle >= 5) gps_cycle = 0;

      // Send GPS meta-packet when: counter reached 0, GPS is enabled in config,
      // and TinyGPS++ reports a valid fix that is not stale.
      bool send_gps_meta = (gps_cycle == 0)
                        && usrConf.gps_en
                        && gps_tx.location.isValid()
                        && gps_tx.location.age() < usrConf.tx_gps_stale_timeout_ms;

      if (send_gps_meta)
      {
        // ---------------------------------------------------------------
        // GPS meta-packet burst (replaces one control packet per 500ms)
        //
        // Step 1: 6-byte announcement.
        // Primes RX to switch radio to implicitHeader(14) before the data arrives.
        // byte3=0xF3 is the meta-packet type marker. byte4=0x01 = GPS upcoming.
        // ---------------------------------------------------------------
        uint8_t announcePkt[6];
        memcpy(announcePkt, usrConf.dest_address, 3);
        announcePkt[3] = 0xF3;
        announcePkt[4] = 0x01;
        announcePkt[5] = esp_crc8(announcePkt, 5);

        rxprint("Sending GPS announcement: ");
        #ifdef DEBUG_RX
        printHexArray(announcePkt, 6);
        #endif

        radio.implicitHeader(6);
        radio.startTransmit(announcePkt, 6);
        num_sent_packets++;
        vTaskDelay(pdMS_TO_TICKS(10));  // wait for 6-byte TX to complete; RX switches mode during this window

        // ---------------------------------------------------------------
        // Step 2: 14-byte GPS data packet.
        // lat/lng as int32_t microdegrees (degrees × 1e6), little-endian.
        // Precision: ±0.111 m — sufficient for Phase B 500 m distance check.
        // ---------------------------------------------------------------
        uint8_t gpsPkt[14];
        memcpy(gpsPkt, usrConf.dest_address, 3);
        gpsPkt[3] = 0xF3;
        gpsPkt[4] = 0x02;  // subtype: GPS coordinate data

        int32_t lat_ud = (int32_t)(gps_tx.location.lat() * 1e6);
        int32_t lng_ud = (int32_t)(gps_tx.location.lng() * 1e6);
        memcpy(gpsPkt + 5, &lat_ud, 4);    // bytes 5–8: latitude microdegrees
        memcpy(gpsPkt + 9, &lng_ud, 4);    // bytes 9–12: longitude microdegrees
        gpsPkt[13] = esp_crc8(gpsPkt, 13); // CRC over bytes 0–12

        rxprint("Sending GPS data: ");
        #ifdef DEBUG_RX
        printHexArray(gpsPkt, 14);
        #endif

        radio.implicitHeader(14);
        radio.startTransmit(gpsPkt, 14);
        // 14-byte packet needs slightly more air time than 6-byte at SF6/BW250
        vTaskDelay(pdMS_TO_TICKS(15));
      }
      else
      {
        // ---------------------------------------------------------------
        // Normal 6-byte control packet
        //
        // THR capped at 0xF2 (242): 0xF3 is the GPS meta-packet marker and
        // must never appear in the THR field of a control packet.
        // 0xF2 = 94.9% max throttle — imperceptible difference from uncapped 95.3%.
        // ---------------------------------------------------------------
        uint8_t sendArray[6];
        memcpy(sendArray, usrConf.dest_address, 3);

        if(system_locked)
        {
          sendArray[3] = 0;
          sendArray[4] = 127;
        }
        else
        {
          uint8_t thr = calcFinalThrottle();
          // V3 - 2026-04-25 - P7: cap at 0xF0 (240=94.1%) to reserve 0xF1-0xFF for all meta-packet types.
          // 0xF1=RTM state, 0xF2=FM override, 0xF3=GPS coord. Was 0xF2 cap before P7.
          sendArray[3] = (thr > 0xF0) ? 0xF0 : thr;
          sendArray[4] = steer_scaled;
        }

        thr_sent   = sendArray[3];
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
      }

      // Common exit for both GPS meta and normal paths:
      // return to 6-byte receive mode and wake waitForTelemetry.
      // After a GPS meta burst, RX sends a normal telemetry reply after processing
      // the GPS data packet — waitForTelemetry will receive it as usual.
      radio.implicitHeader(6);
      rfInterrupt = false;
      radio.startReceive();
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

          // Speed conversion: RX sends speed in km/h; convert to the unit selected in web config.
          // 0xFF = no GPS data sentinel (V3 fix: old V2 sentinel 99 km/h removed — collided with real speed)
          if (rcvArray[3] == 2 && telemetry.foil_speed != 0xFF)
          {
              if (usrConf.speed_src == 1) {
                  // Option 1: GPS RX knots (km/h * 0.539957)
                  telemetry.foil_speed = (uint8_t)(telemetry.foil_speed * 0.539957f);
              } 
              else if (usrConf.speed_src == 4) {
                  // Option 4: GPS RX mph (km/h * 0.621371)
                  telemetry.foil_speed = (uint8_t)(telemetry.foil_speed * 0.621371f);
              }
              // Options 0 (RX km/h), 2 (TX km/h), 3 (TX knots), and 5 (TX mph) 
              // are either untouched here or handled elsewhere by the TX GPS logic.
          }
          // ------------------------------------------

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

// V3 - 2026-04-25 - P7: Queue a 3-burst meta-packet transmission.
// Called from loop task (RTM/FM state machines in RTMState.ino).
// sendData() FreeRTOS task consumes the queue.
// type: 0xF1=RTM state, 0xF2=FM override
// value: for 0xF1: 0=inactive 1=active; for 0xF2: 0-3 FM mode
void queueMetaPacketBurst(uint8_t type, uint8_t value)
{
  rtm_meta_type  = type;
  rtm_meta_value = value;
  rtm_meta_count = 3;  // 3 bursts at 100ms intervals = 300ms total send window
}
