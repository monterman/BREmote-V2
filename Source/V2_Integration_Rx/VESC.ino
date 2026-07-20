// V2.5-Evo - 2026-07-19 - SW56 F1+F2 (Rex CRITICAL + HIGH / Fable F1+F2, applied post-audit): (F1) clamp the vescRelayBuffer relay memcpy to sizeof(vescRelayBuffer) — SW56 raised the guard ceiling 30→48 but vescRelayBuffer is still 34, so a valid-CRC 35–48 B frame (which fa99429 is designed to ACCEPT from newer VESC FW) overflowed a global by up to 14 B into the adjacent volatile motor-command state (thr_received/PWM_active/PWM0_time/PWM1_time) = motor-safety class, NO-GO for field until fixed. (F2, pre-existing) validate the RAW length byte before the uint8_t `eom = raw_message[1]+5` addition — len 251–255 wrapped eom to 0–4, bypassed the guard, and let the payload copy (which uses raw_message[1], not eom) write up to 255 B into the caller's 48 B buffer. Both fixes are bounds-only; no protocol/offset/CRC/mutex change; confStruct/SW_VERSION unchanged
// V2.5-Evo - 2026-07-19 - SW56: receiveFromVESC()/getValuesSelective() RX buffers 30→48 — with VESC_MORE_VALUES the COMM_GET_VALUES_SELECTIVE reply is a 32-byte UART frame (27-byte payload +5 framing), but eom=raw_message[1]+5=32 tripped the 30-byte overflow guard (32>30) and returned 0, rejecting every telemetry reply (latent since the extended mask was enabled; single-value mode's 14-byte frame still fit). 48 comfortably holds it (48 > VESC_PACK_LEN+5=32). Buffer size only; mask/offsets/CRC/echo-validation/mutex unchanged; confStruct/SW_VERSION unchanged
// V2.5-Evo - 2026-05-14 - SW55: rcv_err removed from receiveFromVESC() — flag was never cleared within 200ms window, any stray byte poisoned entire receive attempt; CRC handles frame validation
// V2.5-Evo - 2026-05-14 - SW54: revert SW51/SW52 retry loop — rapid repeated I2C writes caused AW9523 bus corruption at idle; back to single setUartMux(0) + 20ms delay
// V2.5-Evo - 2026-05-14 - SW52: MUX retry count 3→5 for better EMI resilience under sustained motor load
// V2.5-Evo - 2026-05-14 - SW51: setUartMux(0) retried 3× in getVescLoop() — motor EMI corrupts AW9523 I2C writes, single write unreliable under load
// V2.5-Evo - 2026-05-13 - SW50: foil_motor_amps encoded (whole amps, 0–250); added to VESC timeout reset
// V2.5-Evo - 2026-05-13 - SW49: batCur_amps divisor 100000→100 (0.01A scale; typo caused power to always read 0)
// V2.5-Evo - 2026-05-13 - SW45: fbatVolt moved before power calc (was after — foil_power was always 0); last_uart_packet boot guard
// V2.5-Evo - 2026-05-11 - Telemetry Fix: foil_power invalidated on VESC timeout; dead Serial1.flush() removed
// V2.5-Evo - 2026-04-29 - Bundle B: vesc_timeout_s SPIFFS param replaces hardcoded 20s VESC timeout
// V2.5-Evo - 2026-05-06 - Drain Serial1 RX buffer in getVescLoop() to prevent stale GPS NMEA from corrupting VESC frame parsing
// Define the global struct
vesc_struct vesc;

void getVescLoop()
{
  // SW45: start timeout clock from first actual call, not from boot (last_uart_packet=0 caused instant 6s timeout)
  static bool vesc_first_call = true;
  if (vesc_first_call) { last_uart_packet = millis(); vesc_first_call = false; }

  // SW54: single setUartMux(0) + 20ms settle. SW51/SW52 retry loops caused I2C bus
  // corruption at idle (GPS chars=0, VESC zero packets). Longer settle time gives MUX
  // more time to stabilise without hammering AW9523.
  setUartMux(0);
  vTaskDelay(pdMS_TO_TICKS(20));

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
  
  // Use configurable timeout (vesc_timeout_s). Default 6s minimises stale VESC data (range 5-60s; raise toward 12s if the VESC's ~8-9s cold-restart trips a false N/A).
  // If no UART packet received within this window, mark battery and temperature as unavailable.
  if(millis() - last_uart_packet > ((uint32_t)usrConf.vesc_timeout_s * 1000UL))
  {
    telemetry.foil_bat         = 0xFF;
    telemetry.foil_temp        = 0xFF;
    telemetry.foil_power       = 0xFF;
    telemetry.foil_motor_amps  = 0xFF;
    telemetry.foil_voltage     = 0xFF;
    telemetry.foil_duty        = 0xFF;
    telemetry.foil_erpm_lo     = 0xFF;
    telemetry.foil_erpm_hi     = 0xFF;
    telemetry.foil_wh_lo       = 0xFF;
    telemetry.foil_wh_hi       = 0xFF;
  }
}

// buffer_get_float32_auto() is provided by vesc_buffer.cpp (exponent bias 126, not IEEE-754 127).

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
  #define BatVolt   0
  #define WattHours 3

  vesc_command[1] = 0;
  vesc_command[2] = 0;
  vesc_command[3] = (1<<BatVolt);

  #ifdef VESC_MORE_VALUES
    vesc_command[3] |= (1<<WattHours);
    vesc_command[4] = (1<<FET_TEMP) + (1<<MotCurrent) + (1<<BatCurrent) + (1<<Duty) + (1<<ERPM);
  #else
    vesc_command[4] = (1<<FET_TEMP);
  #endif

  sendToVESC(vesc_command, 5, interface);
  
  uint8_t message[48];  // SW56: was 30; matches receiveFromVESC() raw_message so the full 27-byte selective payload copies in with margin
  
  // V2.5-Evo - 2026-07-19 - Version-robust reply validation (works across VESC FW 3.x-7.x).
  // Accept the reply if it ECHOES our exact command id + 4-byte mask AND is at least
  // VESC_PACK_LEN bytes, instead of requiring an EXACT total-length match. VESC only ever
  // APPENDS new fields at the end of the values struct across firmware versions, so a
  // mask-matched reply always carries our requested fields at the known offsets regardless
  // of any trailing bytes a newer firmware adds. A short or mismatched reply is rejected
  // (never misparsed). Telemetry-only path; no motor/safety impact. (Previously == VESC_PACK_LEN,
  // which would silently drop a valid reply if any future VESC FW changed the total length.)
  int vescRxLen = receiveFromVESC(message, interface);
  bool vescReplyValid = (vescRxLen >= VESC_PACK_LEN) &&
                        (message[0] == vesc_command[0]) &&   // COMM_GET_VALUES_SELECTIVE echoed
                        (message[1] == vesc_command[1]) &&   // 4-byte mask echoed back...
                        (message[2] == vesc_command[2]) &&
                        (message[3] == vesc_command[3]) &&
                        (message[4] == vesc_command[4]);
  if(vescReplyValid)
  {
    int32_t cnt = 5;

    // V2.5-Evo fix (Bug 2): guard ALL vesc struct writes inside the mutex block.
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
      #ifdef VESC_MORE_VALUES
        vesc.wh_raw = (int32_t)(buffer_get_float32_auto(message, &cnt) * 10.0f);
      #endif
      xSemaphoreGive(vescMutex);
    }

    // SW45: fbatVolt must be updated before the power block — previous location (after #endif) used stale value
    fbatVolt = (float)vesc.batVolt / 10.0;

    #ifdef VESC_MORE_VALUES
      // Power calculation uses vesc.batCur/erpm which were just written on this same core — no race here
      float batCur_amps = (float)vesc.batCur / 100.0f;
      float watts = fbatVolt * batCur_amps;
      if (watts < 0.0f) watts = 0.0f;
      telemetry.foil_power      = (uint8_t)constrain(watts / 50.0f, 0.0f, 255.0f);
      telemetry.foil_motor_amps = (uint8_t)constrain((float)vesc.motCur / 100.0f, 0.0f, 250.0f);

      telemetry.foil_voltage = (fbatVolt > 0.1f)
          ? (uint8_t)constrain(fbatVolt * 2.0f, 0.0f, 254.0f) : 0xFF;

      float duty_pct = fabsf((float)vesc.duty / 10.0f);
      telemetry.foil_duty = (duty_pct <= 100.0f) ? (uint8_t)(duty_pct) : 0xFF;

      uint16_t erpm_s = (uint16_t)constrain((long)(abs(vesc.erpm) / 100), 0L, 0xFFFEL);
      telemetry.foil_erpm_lo = (uint8_t)(erpm_s & 0xFF);
      telemetry.foil_erpm_hi = (uint8_t)(erpm_s >> 8);

      uint16_t wh_val = (vesc.wh_raw >= 0 && vesc.wh_raw <= 0xFFFE)
                        ? (uint16_t)vesc.wh_raw : 0xFFFF;
      telemetry.foil_wh_lo = (uint8_t)(wh_val & 0xFF);
      telemetry.foil_wh_hi = (uint8_t)(wh_val >> 8);

      #ifdef DEBUG_VESC
      Serial.print("V="); Serial.print(fbatVolt);
      Serial.print(" I="); Serial.print(batCur_amps);
      Serial.print(" RPM="); Serial.print(vesc.erpm);
      Serial.print(" W="); Serial.print(watts);
      Serial.print(" encoded="); Serial.println(telemetry.foil_power);
      #endif
    #endif

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
  uint8_t eom = 48; // SW56: was 30; 32-byte VESC_MORE_VALUES selective frame (eom=raw_message[1]+5=32) overflowed the old 30-byte guard and was rejected. 48 > VESC_PACK_LEN+5
  uint8_t raw_message[eom];
  // V2.5-Evo - 2026-06-07 - Audit #6: zero the buffer so a truncated VESC reply
  // can't read leftover stack at raw_message[eom-1] / via the length byte.
  memset(raw_message, 0, sizeof(raw_message));

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
          // SW55: keep scanning — don't poison the window with a persistent error flag.
          // A stray byte resets the counter; the next 0x02 starts a fresh frame attempt.
          // CRC at the end handles frame integrity.
          VESC_DEBUG_PRINTLN(".");
          cnt=0;
        }
      }
      if(cnt == 2)
      {
          // V2.5-Evo - 2026-07-19 - SW56 F2 (Rex HIGH / Fable F2): validate the RAW length
          // byte BEFORE the uint8_t addition. eom is uint8_t, so raw_message[1] in [251,255]
          // wraps eom to 0-4, silently PASSES the old `eom > sizeof(raw_message)` guard, and
          // the payload copy below (which uses raw_message[1], NOT eom) then writes up to
          // 255 bytes into the caller's 48-byte buffer. Bounding the raw byte to
          // sizeof(raw_message)-5 closes the wraparound AND caps that payload copy.
          if (raw_message[1] > (sizeof(raw_message) - 5)) {
            VESC_DEBUG_PRINTLN("VESC message too long - buffer overflow prevented!");
            return 0;
          }
          eom = raw_message[1] + 5;
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

  // V2.5-Evo - 2026-06-07 - Audit #6: if the length byte never arrived (truncated
  // reply), eom is still 30 and raw_message[eom-1] would read uninitialized stack.
  // Bail before touching the buffer. Telemetry-only path; no motor/safety impact.
  if (cnt < 2) return 0;

  if(raw_message[eom-1] == 3)
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
      // V2.5-Evo - 2026-07-19 - SW56 F1 (Rex CRITICAL / Fable F1): clamp to the DESTINATION
      // size. SW56 raised the guard ceiling 30->48 with raw_message, but vescRelayBuffer is
      // still 34 (BREmote_V2_Rx.h) — so a valid-CRC 35-48 byte frame (exactly what the
      // fa99429 version-robust validation is DESIGNED to accept from newer VESC FW) would
      // overflow it by up to 14 bytes into adjacent globals. In this translation unit those
      // include the volatile motor-command state (thr_received / PWM_active / PWM0_time /
      // PWM1_time), so this was a telemetry path able to corrupt throttle/PWM = the exact
      // class Section 9 forbids. Clamping restores the invariant: guard ceiling <= every dest.
      size_t relayLen = (eom < sizeof(vescRelayBuffer)) ? eom : sizeof(vescRelayBuffer);
      memcpy(&vescRelayBuffer[0], &raw_message[0], relayLen);
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