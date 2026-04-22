void getVescLoop()
{
  setUartMux(0);
  vTaskDelay(pdMS_TO_TICKS(10));
  //Serial1.end();
  //Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  //while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));
  
  // Flush the serial buffer to get fresh data
  Serial1.flush();

  if( getValuesSelective(&Serial1) )
  {
    last_uart_packet = millis();
  }
  get_vesc_timer = millis();

  // Check for VESC connection break
  if(millis() - last_uart_packet > 20000)
  {
    telemetry.foil_bat = 0xFF;
    telemetry.foil_temp = 0xFF;
  }
}

//total 4.5ms
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
  //Byte 3:
  #define BatVolt 0

  vesc_command[1] = 0;
  vesc_command[2] = 0;
  vesc_command[3] = (1<<BatVolt);
  #ifdef VESC_MORE_VALUES
    vesc_command[4] = (1<<FET_TEMP)+ (1<<MotCurrent) + (1<<BatCurrent) + (1<<Duty);
  #else
    vesc_command[4] = (1<<FET_TEMP);
  #endif

  sendToVESC(vesc_command, 5, interface);

  //For the above values, answer should be 19 byte
  uint8_t message[25];
  
  //Is it really 9 or 19 byte?
  if(receiveFromVESC(message, interface) == VESC_PACK_LEN)
  {
    int16_t fetTemp = 0;
    int32_t motCur = 0;
    int32_t batCur = 0;
    int16_t duty = 0;
    int16_t batVolt = 0;

    int32_t cnt = 5; //Dont care about the Mask    
    fetTemp = buffer_get_int16(message, &cnt); 
    #ifdef VESC_MORE_VALUES
      motCur = buffer_get_int32(message, &cnt);
      batCur = buffer_get_int32(message, &cnt);
      duty = buffer_get_int16(message, &cnt);

      // Power calculation
      // batCur unit: int32 in mA*100. Divide by 100000.0f to get Amps.
      float batCur_amps = (float)batCur / 100000.0f;
      // fbatVolt is global float set at end of this function from previous cycle (one-cycle lag, acceptable)
      float watts = fbatVolt * batCur_amps;
      if (watts < 0.0f) watts = 0.0f;  // Clamp: ignore regenerative braking (negative current)
      // SCALE: foil_power = watts/50  |  DECODE on Tx Display.ino: watts = foil_power * 50  (range 0-12750W)
      telemetry.foil_power = (uint8_t)constrain(watts / 50.0f, 0.0f, 255.0f);

      #ifdef DEBUG_VESC
      Serial.print("V="); Serial.print(fbatVolt);
      Serial.print(" I="); Serial.print(batCur_amps);
      Serial.print(" W="); Serial.print(watts);
      Serial.print(" encoded="); Serial.println(telemetry.foil_power);
      #endif
    #endif
    batVolt = buffer_get_int16(message, &cnt);
    
    fbatVolt = (float)batVolt / 10.0;
    telemetry.foil_bat = getUbatPercent(fbatVolt);
    telemetry.foil_temp = (uint8_t)(fetTemp / 10);

    //String sprt = "Fet Temp: " + String(fetTemp) + "°C, Mot. Curr.: " + String(motCur) + "A, Inp. Curr: " + String(batCur) + "A, Duty:: " + String(duty) + "%, Volt: " + String(batVolt) + "V";
    //Serial.println(sprt);
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
  uint8_t eom = 25;
  uint8_t raw_message[eom];
  bool rcv_err = 0;

  unsigned long started = millis();

  //Read until whole palyoad ist received
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
          rcv_err = true;  // FIX: Set error flag when start byte is wrong
          cnt=0;
        }
      }
      if(cnt == 2)
      {
          eom = raw_message[1] + 5; //5 byte overhead
          
          // SECURITY FIX: Validate message length doesn't exceed buffer size
          if (eom > sizeof(raw_message)) {
            VESC_DEBUG_PRINTLN("VESC message too long - buffer overflow prevented!");
            return 0;
          }
      }
    }
  }
  
#ifdef DEBUG_VESC
  Serial.println();
  Serial.println();

  for(int i = 0; i < 25; i++)
  {
    Serial.print(raw_message[i], HEX);
    Serial.print(" ");
  }

  Serial.println();
  Serial.println();
#endif

  //Check if End is reached
  if(!rcv_err && raw_message[eom-1] == 3)
  {
    uint16_t crcMessage = 0;
    uint16_t crcPayload = 0;

    crcMessage = raw_message[eom - 3] << 8;
    crcMessage &= 0xFF00;
    crcMessage += raw_message[eom - 2];

    //Extract payload and also calculate CRC
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
