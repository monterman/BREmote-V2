// V3 - 2026-04-24 - Added GPS meta-packet reception: gps_meta_pending state, processMetaGpsPacket(), triggeredReceive() 2-path state machine

void radioErrorHalt(int type)
{
  if(type == 1) while(1) blinkErr(2, AP_L_BIND);
  while(1) blinkErr(3, AP_L_BIND);
}

void radioInitSuccess()
{
  // No extra init needed on RX side
}

void startupRadio()
{
  initRadioHardware();
}

void ICACHE_RAM_ATTR packetReceived(void)
{
  if(rxIsrState && !rfInterrupt)
  {
    rfInterrupt = true;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(triggerReceiveSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  else
  {
    rfInterrupt = true;
  }
}

// Function for waiting node to pair
bool waitForPairing()
{
  usrConf.paired = false;

  // First check for address conflicts
  if (0)//!checkAndAdjustAddress())
  {
    return false;  // Too many conflicts
  }

  uint8_t responsePacket[8];
  unsigned long startTime = millis();

  while (millis() - startTime < PAIRING_TIMEOUT)
  {
    radio.implicitHeader(5);
    radio.startReceive();
    rxprintln("Waiting for pairing packet...");
    uint8_t buffer[15];

    while(!rfInterrupt && millis() - startTime < PAIRING_TIMEOUT) blinkBind(2);
    delay(10);
    if (rfInterrupt && radio.readData(buffer, 15) == RADIOLIB_ERR_NONE)
    {
      rfInterrupt = false;
      rxprintln("Received response");
      #ifdef DEBUG_RX
      printHexArray(buffer, 15);
      #endif

      if (buffer[0] == 0xAB)
      {
        rxprintln("1st byte matches");
        // Verify CRC of received packet directly
        uint8_t receivedCRC = buffer[4];
        uint8_t calculatedCRC = esp_crc8(buffer, 4);

        if (receivedCRC == calculatedCRC)
        {
          rxprintln("CRC ok");
          // Store potential partner's address
          uint8_t temp_addr[3];
          memcpy(temp_addr, buffer + 1, 3);

          // Prepare response packet
          responsePacket[0] = 0xBA;
          memcpy(responsePacket + 1, temp_addr, 3);
          memcpy(responsePacket + 4, usrConf.own_address, 3);

          // Calculate CRC;
          responsePacket[7] = esp_crc8(responsePacket, 7);

          delay(100);
          rxprintln("Sending response: ");
          #ifdef DEBUG_RX
          printHexArray(responsePacket, 8);
          #endif
          // Send response
          radio.implicitHeader(8);
          radio.startTransmit(responsePacket, 8);
          delay(10);
          radio.startReceive();
          rfInterrupt = false;

          while (millis() - startTime < PAIRING_TIMEOUT)
          {
            while(!rfInterrupt && millis() - startTime < PAIRING_TIMEOUT) delay(10);
            delay(10);
            if (rfInterrupt && radio.readData(buffer, 15) == RADIOLIB_ERR_NONE)
            {
              rfInterrupt = false;
              rxprintln("Received response");
              if (buffer[0] == 0xAC && memcmp(buffer + 1, usrConf.own_address, 3) == 0)
              {
                rxprintln("Address matches");
                // Verify CRC of final packet directly
                receivedCRC = buffer[7];
                calculatedCRC = esp_crc8(buffer, 7);

                if (receivedCRC == calculatedCRC)
                {
                  rxprintln("CRC ok, pairing success");
                  // Save partner's address
                  memcpy(usrConf.dest_address, buffer + 4, 3);
                  usrConf.paired = true;
                  saveConfToSPIFFS(usrConf);
                  aw.digitalWrite(AP_L_BIND, LOW);
                  return true;
                }
              }
            }
            else rfInterrupt = false;
          }
        }
      }
    }
    else rfInterrupt = false;
  }
  return false;
}

// V3 - 2026-04-24 - GPS meta-packet state and handler for 0xF3 protocol

// gps_meta_pending: set true when a 0xF3 announcement (6-byte) is received.
// On the NEXT wakeup of triggeredReceive, read 14 bytes (GPS data) instead
// of the normal 6-byte control packet.
static bool gps_meta_pending = false;

// ============================================================
// processMetaGpsPacket - Decode a received 14-byte GPS data packet
// ============================================================
//
// What it does:
//   Validates destination address, CRC8 (over bytes 0-12 stored in byte 13),
//   packet type (0xF3) and subtype (0x02). On success, extracts TX lat/lng
//   stored as int32_t microdegrees (little-endian) and writes the three
//   rx_tx_gps_* globals declared in BREmote_V2_Rx.h.
//
// Inputs:
//   pkt - pointer to a 14-byte buffer containing the received GPS data packet
//
// Side effects:
//   On success: updates rx_tx_gps_lat, rx_tx_gps_lng, rx_tx_gps_timestamp.
//   Always: prints diagnostics to Serial.
// ============================================================
static void processMetaGpsPacket(uint8_t *pkt)
{
  if (memcmp(pkt, usrConf.own_address, 3) != 0)
  {
    rxprintln("META GPS: address mismatch, discarding");
    return;
  }

  // CRC covers bytes 0-12; result stored in byte 13
  if (pkt[13] != esp_crc8(pkt, 13))
  {
    rxprintln("META GPS: CRC fail, discarding");
    return;
  }

  if (pkt[3] != 0xF3 || pkt[4] != 0x02)
  {
    rxprintln("META GPS: unexpected type/subtype, discarding");
    return;
  }

  // Extract lat/lng as int32_t microdegrees stored little-endian.
  // memcpy avoids strict-aliasing UB that a direct pointer cast would cause.
  int32_t lat_ud, lng_ud;
  memcpy(&lat_ud, pkt + 5, 4);
  memcpy(&lng_ud, pkt + 9, 4);

  rx_tx_gps_lat       = (double)lat_ud / 1e6;
  rx_tx_gps_lng       = (double)lng_ud / 1e6;
  rx_tx_gps_timestamp = millis();

  Serial.printf("META GPS received: lat=%.6f lng=%.6f\n",
                rx_tx_gps_lat, rx_tx_gps_lng);
}

void triggeredReceive(void *parameter) {
  while (1)
  {
    // Wait for semaphore given by packetReceived() ISR on any DIO1 event
    if (xSemaphoreTake(triggerReceiveSemaphore, portMAX_DELAY) == pdTRUE)
    {
      if (gps_meta_pending)
      {
        // ---- GPS data packet path (14 bytes) ----
        // A 0xF3 announcement arrived on the previous wakeup.
        // Radio is already in implicitHeader(14) + startReceive mode.
        // TX has sent the 14-byte GPS coordinate packet; read and decode it.
        gps_meta_pending = false;  // clear before any early return

        uint8_t gpsArray[14];
        if (radio.readData(gpsArray, 14) == RADIOLIB_ERR_NONE)
        {
          processMetaGpsPacket(gpsArray);
        }
        else
        {
          rxprintln("META GPS: readData error");
        }
        // Fall through to common exit below (implicitHeader(6) + startReceive + rfInterrupt=false)
      }
      else
      {
        // ---- Normal 6-byte control packet path ----
        uint8_t rcvArray[6];
        if (radio.readData(rcvArray, 6) == RADIOLIB_ERR_NONE)
        {
          rxprint("Received packet: ");
          #ifdef DEBUG_RX
          printHexArray(rcvArray, 6);
          #endif

          if (memcmp(rcvArray, usrConf.own_address, 3) == 0)
          {
            rxprintln("Address matches");

            if (rcvArray[5] == esp_crc8(rcvArray, 5))
            {
              rxprintln("CRC ok");

              if (rcvArray[3] == 0xF3)
              {
                // ---- GPS announcement ----
                // TX will send a 14-byte GPS data packet ~10ms from now.
                // Switch radio to 14-byte mode immediately — SPI writes take
                // <2ms, so we will be ready well before the GPS data arrives.
                rxprintln("META GPS: announcement, switching to 14-byte mode");
                gps_meta_pending = true;
                radio.implicitHeader(14);
                radio.startReceive();
                rfInterrupt = false;
                // TX does not expect a telemetry reply here; skip the common exit.
                continue;
              }

              // ---- Normal throttle/steering control packet ----
              last_packet = millis();
#ifdef WIFI_ENABLED
              webCfgNotifyRxConnected();
#endif
              rxprint("RSSI: ");
              rxprint(radio.getRSSI());
              rxprint(", SNR: ");
              rxprintln(radio.getSNR());

              thr_received      = rcvArray[3];
              steering_received = rcvArray[4];

              telemetry.link_quality = getLinkQuality(radio.getRSSI(), radio.getSNR());

              rxprintln("Sending response");

              uint8_t sendArray[6];
              memcpy(sendArray, usrConf.dest_address, 3);
              uint8_t* ptr = (uint8_t*)&telemetry;
              sendArray[3] = telemetry_index;
              sendArray[4] = ptr[telemetry_index];
              telemetry_index++;
              if(telemetry_index >= sizeof(TelemetryPacket))
              {
                telemetry_index = 0;
              }
              sendArray[5] = esp_crc8(sendArray, 5);

              #ifdef DEBUG_RX
              printHexArray(sendArray, 6);
              #endif

              vTaskDelay(pdMS_TO_TICKS(10));
              radio.implicitHeader(6);
              radio.startTransmit(sendArray, 6);
              vTaskDelay(pdMS_TO_TICKS(10));
            }
          }
        }
        else
        {
          rxprintln("Rx err");
        }
      }

      // Common exit: restore 6-byte receive mode and clear stale interrupt flag.
      // The GPS announcement path uses 'continue' and never reaches here.
      radio.implicitHeader(6);
      radio.startReceive();
      rfInterrupt = false;
    }
  }
}

// getLinkQuality() is now in ../Common/RadioCommon.h
