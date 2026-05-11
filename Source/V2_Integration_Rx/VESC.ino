// V2.5-Evo - 2026-05-11 - Telemetry Fix: foil_power invalidated on VESC timeout; dead Serial1.flush() removed
// V3 - 2026-04-29 - Bundle B: vesc_timeout_s SPIFFS param replaces hardcoded 20s VESC timeout
// V2.5-Evo - 2026-05-06 - Drain Serial1 RX buffer in getVescLoop() to prevent stale GPS NMEA from corrupting VESC frame parsing
// Define the global struct
vesc_struct vesc;

void getVescLoop()
{
  setUartMux(0);
  vTaskDelay(pdMS_TO_TICKS(10));

  // V2.5-Evo - 2026-05-06 - RX-buffer drain before VESC query.
  // Serial1 is shared with GPS via an AW9523-controlled analog mux. After mux
  // switch from GPS to VESC, the Serial1 RX buffer still contains partial NMEA
  // sentences. Without this drain, those bytes prefix the VESC response and
  // corrupt frame parsing in receiveFromVESC(), causing every VESC packet to be
  // rejected and last_uart_packet never to update. Verified by ?vescping showing
  // pkt_age_ms growing unboundedly before this fix. Non-blocking: only drains
  // bytes already in the buffer, never blocks waiting for new ones.
  while (Serial1.available()) Serial1.read();

  if( getValuesSelective(&Serial1) )
  {
    last_uart_packet = millis();
    vesc.last_packet = last_uart_packet;
  }
  get_vesc_timer = millis();
  
  // Use configurable timeout (vesc_timeout_s). Default 12s gives margin above ~8-9s VESC cold-restart time.
  // If no UART packet received within this window, mark battery and temperature as unavailable.
  if(millis() - last_uart_packet > ((uint32_t)usrConf.vesc_timeout_s * 1000UL))
  {
    telemetry.foil_bat   = 0xFF;
    telemetry.foil_temp  = 0xFF;
    telemetry.foil_power = 0xFF;
  }
}

bool getValuesSelective(Stream* interface)
{
  uint8_t vesc_command[5];
  vesc_command[0] = COMM_GET_VALUES_SELECTIVE;
  
  //Mask is 32 bit, divided in 4 byte, see "commands.c", line 377 in the VESC Firmware
  //Byte 4:
  #define FET_TEMP 0
  #define MotCurrent 2
  #define BatCurrent 3
  #define Duty 6
  #define ERPM 7
  
  //Byte 3:
  #define BatVolt 0

  vesc_command[1] = 0;
  vesc_command[2] = 0;
  vesc_command[3] = (1<<BatVolt);
  
  #ifdef VESC_MORE_VALUES
    vesc_command[4] = (1<<FET_TEMP) + (1<<MotCurrent) + (1<<BatCurrent) + (1<<Duty) + (1<<ERPM);
  #else
    vesc_command[4] = (1<<FET_TEMP);
  #endif

  sendToVESC(vesc_command, 5, interface);
  
  uint8_t message[30];
  
  if(receiveFromVESC(message, interface) == VESC_PACK_LEN)
  {
    int32_t cnt = 5;

    // V3 fix (Bug 2): guard ALL vesc struct writes inside the mutex block.
    // convertToLogData() on Core 0 (loggerTask) reads these same fields simultaneously.
    // If the take times out (50ms), skip this packet's struct update entirely —
    // one missed update is safer than a torn cross-core write.
    extern SemaphoreHandle_t vescMutex;
    if (vescMutex && xSemaphoreTake(vescMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      vesc.fetTemp = buffer_get_int16(message, &cnt);

      #ifdef VESC_MORE_VALUES
        vesc.motCur = buffer_get_int32(message, &cnt);
        vesc.batCur = buffer_get_int32(message, &cnt);
        vesc.duty   = buffer_get_int16(message, &cnt);
        vesc.erpm   = buffer_get_int32(message, &cnt);
      #endif

      vesc.batVolt = buffer_get_int16(message, &cnt);
      xSemaphoreGive(vescMutex);
    }

    #ifdef VESC_MORE_VALUES
      // Power calculation uses vesc.batCur/erpm which were just written on this same core — no race here
      float batCur_amps = (float)vesc.batCur / 100000.0f;
      float watts = fbatVolt * batCur_amps;
      if (watts < 0.0f) watts = 0.0f;
      telemetry.foil_power = (uint8_t)constrain(watts / 50.0f, 0.0f, 255.0f);

      #ifdef DEBUG_VESC
      Serial.print("V="); Serial.print(fbatVolt);
      Serial.print(" I="); Serial.print(batCur_amps);
      Serial.print(" RPM="); Serial.print(vesc.erpm);
      Serial.print(" W="); Serial.print(watts);
      Serial.print(" encoded="); Serial.println(telemetry.foil_power);
      #endif
    #endif

    fbatVolt = (float)vesc.batVolt / 10.0;
    telemetry.foil_bat = getUbatPercent(fbatVolt);
    telemetry.foil_temp = (uint8_t)(vesc.fetTemp / 10);

    return 1;
  }
  else
  {
    return 0;
  }
}

int receiveFromVESC(uint8_t * buf, Stream* interface)
{
  uint8_t cnt = 0;
  uint8_t eom = 30; // Increased buffer max
  uint8_t raw_message[eom];
  bool rcv_err = 0;

  unsigned long started = millis();

  while( ((millis() - started) < 200) && cnt != eom)
  {
    if(interface->available())
    {
      raw_message[cnt++] = interface->read();
      if(cnt == 1)
      {
        if(raw_message[0] != 2)
        {
          VESC_DEBUG_PRINTLN(".");
          rcv_err = true;  
          cnt=0;
        }
      }
      if(cnt == 2)
      {
          eom = raw_message[1] + 5;
          if (eom > sizeof(raw_message)) {
            VESC_DEBUG_PRINTLN("VESC message too long - buffer overflow prevented!");
            return 0;
          }
      }
    }
  }
  
#ifdef DEBUG_VESC
  Serial.println();
  for(int i = 0; i < 25; i++)
  {
    Serial.print(raw_message[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  if(!rcv_err && raw_message[eom-1] == 3)
  {
    uint16_t crcMessage = 0;
    uint16_t crcPayload = 0;

    crcMessage = raw_message[eom - 3] << 8;
    crcMessage &= 0xFF00;
    crcMessage += raw_message[eom - 2];
    
    memcpy(buf,&raw_message[2],raw_message[1]);
    crcPayload = vesc_crc16(buf, raw_message[1]);
    
    if(crcPayload == crcMessage)
    {
      memcpy(&vescRelayBuffer[0], &raw_message[0], eom);
      return raw_message[1];
    }
    else
    {
      VESC_DEBUG_PRINTLN("CRC NOK");
      return 0;
    }
  }
  else
  {
    VESC_DEBUG_PRINTLN("Message Error");
    return 0;
  }
}

void sendToVESC(uint8_t * content, int len, Stream* interface)
{
  uint16_t crc = vesc_crc16(content, len);
  uint8_t tosend[16];
  int cnt = 0;
  tosend[cnt++] = 2;
  tosend[cnt++] = len;
  memcpy(tosend + cnt, content, len);
  cnt += len;
  tosend[cnt++] = (uint8_t)(crc >> 8);
  tosend[cnt++] = (uint8_t)(crc & 0xFF);
  tosend[cnt++] = 3;
  interface->write(tosend, cnt);
}