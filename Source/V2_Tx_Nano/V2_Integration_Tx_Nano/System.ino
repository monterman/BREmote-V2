void enterSetup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("");
  Serial.println("Entering Setup...");  

  Serial.println("**************************************");
  Serial.println("**          BREmote V2 TX           **");
  Serial.printf("**        MAC: %012llX         **\n", ESP.getEfuseMac());
  Serial.print("**     HW Identifier: "); Serial.print(checkHWConfig()); Serial.println("       **");
  Serial.printf("**          SW Version: %-10d  **\n", SW_VERSION);
  Serial.printf("**  Compiled: %s %s  **\n", __DATE__, __TIME__);
  Serial.println("**************************************");
}

void exitSetup()
{
  Serial.println("");
  Serial.println("...Leaving Setup");
  Serial.println("");
}

String checkHWConfig()
{
  //Not sure why this is neccessary, otherwise pullup is too stron
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
  digitalWrite(18, LOW);

  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);
  digitalWrite(19, LOW);

  uint8_t pin_id = 0;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  pin_id |= (digitalRead(18)&0x01)<<0;
  pin_id |= (digitalRead(19)&0x01)<<4;

  pinMode(18, INPUT_PULLDOWN);
  pinMode(19, INPUT_PULLDOWN);

  pin_id |= (digitalRead(18)&0x01)<<1;
  pin_id |= (digitalRead(19)&0x01)<<5;

  if(pin_id == 0x11)
  {
    return "Unknown";
  }

  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);

  pin_id |= (digitalRead(19)&0x01)<<6;

  digitalWrite(18, LOW);

  pin_id |= (digitalRead(19)&0x01)<<7;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, OUTPUT);
  digitalWrite(19, HIGH);

  pin_id |= (digitalRead(18)&0x01)<<2;

  digitalWrite(19, LOW);

  pin_id |= (digitalRead(18)&0x01)<<3;

  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);

  //Serial.println(pin_id, HEX);

  switch (pin_id)
  {
    case 0x77:
      // b8
      return "Beta_8M";
      break;
    case 0x44:
      // b9
      return "Beta_9M";
      break;
    case 0x40:
      // g18
      return "Gen1_8M";
      break;
    case 0x04:
      // g19
      return "Gen1_9M";
      break;
    case 0xF7:
      // g28
      return "Gen2_8M";
      break;
    case 0x7F:
      // g29
      return "Gen2_9M";
      break;
    case 0xFF:
      // o18
      return "Diy1_8M";
      break;
    case 0x00:
      // o19
      return "Diy1_9M";
      break;
    case 0x0F:
      // o28
      return "Diy2_8M";
      break;
    case 0xF0:
      // o29
      return "Diy2_9M";
      break;
  
    default:
      return "Unknown";
      break;
  }
}

void printHexArray(const uint8_t* buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // Print leading zero for values less than 0x10
    if (buffer[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(buffer[i], HEX);
    // Add space between bytes (except after the last one)
    if (i < size - 1) {
      Serial.print(" ");
    }
  }
  // Add newline at the end
  Serial.println();
}

void printHexArray16(const volatile uint16_t* buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    // Print leading zeros for values less than 0x1000 and 0x100
    if (buffer[i] < 0x1000) {
      Serial.print("0");
    }
    if (buffer[i] < 0x100) {
      Serial.print("0");
    }
    if (buffer[i] < 0x10) {
      Serial.print("0");
    }
    Serial.print(buffer[i], HEX);
    // Add space between values (except after the last one)
    if (i < size - 1) {
      Serial.print(" ");
    }
  }
  // Add newline at the end
  Serial.println();
}

uint8_t esp_crc8(uint8_t *data, uint8_t length) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void checkStartupButtons()
{
  if(thr_scaled > 100)
  {
    if(tog_input == 1)
    {
      //Delete SPIFFS
      
    }
    else if (tog_input == -1)
    {
      //USB Mode
      Serial.println("USB Mode, type '?' for info...");
      while(1)
      {
        checkSerial();
      }
    }
  }
  else
  {
    if(tog_input == 1)
    {
      //Pairing
      usrConf.paired = 0;
      checkPairing();
    }
    else if (tog_input == -1)
    {
      //Calib
      usrConf.cal_ok = 0;
      checkCal();
    }
  }
}

void checkSerial()
{
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Read input until newline

    // Trim leading and trailing spaces
    command.trim();
    
    // Process the command
    if (command.startsWith("?")) {
      if (command == "?conf") {
        serPrintConf();  // Call function for ?conf
      } 
      else if (command.startsWith("?setConf")) {
        String data = command.substring(command.indexOf(":") + 1);  // Extract everything after ":"
        serSetConf(data);  // Call function for ?setconf with the string after ":"
      } 
      else if (command == "?clearSPIFFS") {
        serClearConf();  // Call function for ?clearSPIFFS
      }
      else if (command == "?applyConf") {
        serApplyConf();  // Call function for ?clearSPIFFS
      }
      else if (command == "?reboot")
      {
        Serial.println("Rebooting now...");
        delay(1000);
        ESP.restart();
      }
      else if (command == "?exitChg")
      {
        Serial.println(" Exit by user");
        exitChargeScreen = 1;
      }
      else if(command == "?printRSSI")
      {
        serPrintRSSI();
      }
      else if(command == "?printInputs")
      {
        serPrintInputs();
      }
      else if(command == "?printTasks")
      {
        serPrintTasks();
      }
      else if(command == "?printPackets")
      {
        serPrintPackets();
      }
      else if (command == "?") {
        // List all possible inputs
        Serial.println("Possible commands:");
        Serial.println("?conf - print info, usrConf");
        Serial.println("?setConf:<data> - write B64 to SPIFFS");
        Serial.println("?applyConf - read conf from SPIFFS and write to usrConf");
        Serial.println("?clearSPIFFS - Clear usrConf from SPIFFS");
        Serial.println("?reboot - Reboot the remote");
        Serial.println("?exitChg - Exit the charge screen");
        Serial.println("?printRSSI - Print RSSI and SNR values until sent 'quit'");
        Serial.println("?printInputs - Print raw input values until sent 'quit'");
        Serial.println("?printTasks - Print task stack usage until sent 'quit'");
        Serial.println("?printPackets - Print current state of tx,rx packets and relation");
      }
      else {
        Serial.println("Unknown command. Type '?' for help.");
      }
    }
    else {
      Serial.println("Unknown command. Type '?' for help.");
    }
  }
}

void serApplyConf()
{
  Serial.print("Reading conf from SPIFFS and applying to usrConf");
  readConfFromSPIFFS(usrConf);
}

void serSetConf(String data) {
  Serial.print("Setting configuration to: ");
  Serial.println(data);
  
  uint8_t* encodedData = new uint8_t[data.length()];
  // Call the function to fill the encodedData array
  for (size_t i = 0; i < data.length(); i++) {
    encodedData[i] = data[i];  // Convert each character to uint8_t
  }

  // Save to SPIFFS
  File file = SPIFFS.open(CONF_FILE_PATH, FILE_WRITE);
  if (!file) {
      Serial.println("Failed to open file for writing");
      delete[] encodedData;
      return;
  }
  file.write(encodedData, data.length());
  file.close();
  Serial.println("Struct saved to SPIFFS as Base64");
  delete[] encodedData;
}

void serClearConf()
{
  Serial.println("Deleting conf from SPIFFS");
  deleteConfFromSPIFFS();
}

void serPrintTasks()
{
  while (true) 
  {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read the input command
        input.trim(); // Remove any whitespace or newline characters
        
        // If the received command matches the stop command, exit the loop
        if (input.equals("quit")) {
            Serial.println("Stopping print loop.");
            break;
        }
    }

    Serial.println("\n=== Task Stack Usage ===");

    Serial.printf("sendData stack left: %u words\n", uxTaskGetStackHighWaterMark(sendDataHandle));
    Serial.printf("telemetry stack left: %u words\n", uxTaskGetStackHighWaterMark(triggeredWaitForTelemetryHandle));
    Serial.printf("measBufCalc stack left: %u words\n", uxTaskGetStackHighWaterMark(measBufCalcHandle));
    Serial.printf("bargraph stack left: %u words\n", uxTaskGetStackHighWaterMark(updateBargraphsHandle));
    Serial.printf("loop() stack left: %u words\n", uxTaskGetStackHighWaterMark(loopTaskHandle));


    Serial.println("========================\n");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void serPrintPackets()
{
  Serial.print("Sent: ");
  Serial.println(num_sent_packets);
  Serial.print("Received: ");
  Serial.println(num_rcv_packets);
  Serial.print("Ratio: ");
  Serial.print(((float)num_rcv_packets/(float)num_sent_packets)*100);
  Serial.println(" %");
}

void serPrintRSSI()
{
  while (true) 
  {
    // Check if data is available on Serial
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read the input command
        input.trim(); // Remove any whitespace or newline characters
        
        // If the received command matches the stop command, exit the loop
        if (input.equals("quit")) {
            Serial.println("Stopping print loop.");
            break;
        }
    }
    
    // Print the variable
    if(millis()-last_packet < 1000)
    {
      Serial.print("RSSI: ");
      Serial.print(radio.getRSSI());
      Serial.print(", SNR: ");
      Serial.println(radio.getSNR());
      
    }
    else
    {
      Serial.print("Failsafe since (ms) ");
      Serial.println(millis()-last_packet);
    }
    delay(50);
  }
}

void serPrintInputs()
{
  while (true) 
  {
    // Check if data is available on Serial
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read the input command
        input.trim(); // Remove any whitespace or newline characters
        
        // If the received command matches the stop command, exit the loop
        if (input.equals("quit")) {
            Serial.println("Stopping print loop.");
            break;
        }
    }
    
    
    Serial.print("Throttle: ");
    Serial.print(thr_scaled);
    Serial.print(", Steering: ");
    Serial.println(steer_scaled);
    delay(50);
  }
}

void serPrintConf()
{
  Serial.println("**************************************");
  Serial.println("**          BREmote V2 TX           **");
  Serial.printf("**        MAC: %012llX         **\n", ESP.getEfuseMac());
  Serial.print("**     HW Identifier: "); Serial.print(checkHWConfig()); Serial.println("       **");
  Serial.printf("**          SW Version: %-10d  **\n", SW_VERSION);
  Serial.printf("**  Compiled: %s %s  **\n", __DATE__, __TIME__);
  Serial.println("**************************************");

  // Read file from SPIFFS
  File file = SPIFFS.open(CONF_FILE_PATH, FILE_READ);
  if (!file) {
      Serial.println("Failed to open file for reading");
  }

  String encodedString = file.readString();
  Serial.println("Encoded Data Read: " + encodedString);
  file.close();

  printConfStruct(usrConf);
}

uint8_t chg_err_cnt = 0;
void checkCharger()
{
  Serial.print("Checking if charging...");
  uint8_t chargeAnimationPos = 0;

  while(!exitChargeScreen)
  {
    ads.startADCReading(MUX_BY_CHANNEL[P_CHGSTAT],false);
    while(!ads.conversionComplete()) delay(1);
    uint16_t chgstat = ads.getLastConversionResults();

    ads.startADCReading(MUX_BY_CHANNEL[P_UBAT_MEAS],false);
    while(!ads.conversionComplete()) delay(1);
    uint16_t bat_volt = ads.getLastConversionResults();
    uint16_t c_bat_volt = (uint16_t)((float)bat_volt * usrConf.ubat_cal * 100.0);

    if(chgstat < 660)
    {
      //Not charging
      Serial.println(" Done");
      break;
    }
    else if(chgstat > 4000 && chgstat < 6600)
    {
      //Charging

      if(chargeAnimationPos == 0)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 0);
        aw.analogWrite(AP_LR3, 1);
        aw.analogWrite(AP_LR4, 0);
        aw.analogWrite(AP_LR5, 0);

        aw.analogWrite(AP_LL1, 0);
        aw.analogWrite(AP_LL2, 0);
        aw.analogWrite(AP_LL3, 0);
        aw.analogWrite(AP_LL4, 0);
        aw.analogWrite(AP_LL5, 0);

        if(c_bat_volt > 350) aw.analogWrite(AP_LL1, 1);
        delay(200);
        if(c_bat_volt > 370) aw.analogWrite(AP_LL2, 1);
      }
      else if(chargeAnimationPos == 1)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 1);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 1);
        aw.analogWrite(AP_LR5, 0);
        if(c_bat_volt > 380) aw.analogWrite(AP_LL3, 1);
      }
      else if(chargeAnimationPos == 2)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 1);
        aw.analogWrite(AP_LR2, 0);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 0);
        aw.analogWrite(AP_LR5, 1);
        if(c_bat_volt > 390) aw.analogWrite(AP_LL4, 1);
      }
      else
      {
        chargeAnimationPos = 0;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 1);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 1);
        aw.analogWrite(AP_LR5, 0);
        if(c_bat_volt > 410) aw.analogWrite(AP_LL5, 1);
      }

      checkSerial();
      delay(200);
    }
    else if(chgstat > 7500 && chgstat < 12000)
    {
      if(chargeAnimationPos == 0)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 0);
        aw.analogWrite(AP_LR3, 1);
        aw.analogWrite(AP_LR4, 0);
        aw.analogWrite(AP_LR5, 0);

        aw.analogWrite(AP_LL1, 1);
        aw.analogWrite(AP_LL2, 1);
        aw.analogWrite(AP_LL3, 1);
        aw.analogWrite(AP_LL4, 1);
        aw.analogWrite(AP_LL5, 1);
      }
      else if(chargeAnimationPos == 1)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 1);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 1);
        aw.analogWrite(AP_LR5, 0);
      }
      else if(chargeAnimationPos == 2)
      {
        chargeAnimationPos++;
        aw.analogWrite(AP_LR1, 1);
        aw.analogWrite(AP_LR2, 0);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 0);
        aw.analogWrite(AP_LR5, 1);
      }
      else
      {
        chargeAnimationPos = 0;
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 1);
        aw.analogWrite(AP_LR3, 0);
        aw.analogWrite(AP_LR4, 1);
        aw.analogWrite(AP_LR5, 0);
      }
      delay(200);
    }
    else
    {
      chg_err_cnt++;
      Serial.print("Count: ");
      Serial.println(chg_err_cnt);
      Serial.print("Stat: ");
      Serial.println(chgstat);
      delay(10);
      if(chg_err_cnt > 10)
      {
        Serial.println("CHG ERR!");
        Serial.print("Stat: ");
        Serial.println(chgstat);
        while(1) displayError(3);
      }
    }
  }
}

void printConfStruct(const confStruct &data) {
    Serial.println("Configuration Struct Values:");
    Serial.print("Version: "); Serial.println(data.version);

    Serial.print("Radio Preset: "); Serial.println(data.radio_preset);
    Serial.print("RF Power: "); Serial.println(data.rf_power);

    Serial.print("Calibration OK: "); Serial.println(data.cal_ok);
    Serial.print("Calibration Offset: "); Serial.println(data.cal_offset);

    Serial.print("Throttle Idle: "); Serial.println(data.thr_idle);
    Serial.print("Throttle Pull: "); Serial.println(data.thr_pull);

    Serial.print("Toggle Left: "); Serial.println(data.tog_left);
    Serial.print("Toggle Middle: "); Serial.println(data.tog_mid);
    Serial.print("Toggle Right: "); Serial.println(data.tog_right);

    Serial.print("Toggle Deadzone: "); Serial.println(data.tog_deadzone);
    Serial.print("Toggle Difference: "); Serial.println(data.tog_diff);
    Serial.print("Toggle Block Time: "); Serial.println(data.tog_block_time);
    
    Serial.print("Trigger Unlock Timeout: "); Serial.println(data.trig_unlock_timeout);
    Serial.print("Lock Wait Time: "); Serial.println(data.lock_waittime);
    Serial.print("Gear Change Wait Time: "); Serial.println(data.gear_change_waittime);
    Serial.print("Gear Display Time: "); Serial.println(data.gear_display_time);
    Serial.print("Menu Timeout: "); Serial.println(data.menu_timeout);
    Serial.print("Error Delete Time: "); Serial.println(data.err_delete_time);

    Serial.print("No Lock: "); Serial.println(data.no_lock);
    Serial.print("No Gear: "); Serial.println(data.no_gear);
    Serial.print("Max Gears: "); Serial.println(data.max_gears);
    Serial.print("Start Gear: "); Serial.println(data.startgear);
    Serial.print("Steer Enabled: "); Serial.println(data.steer_enabled);

    Serial.print("Battery Calibration (Ubat Cal): "); Serial.println(data.ubat_cal, 15); // Floating-point precision

    Serial.print("Paired: "); Serial.println(data.paired);

    Serial.print("Own Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(data.own_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    Serial.print("Destination Address: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(data.dest_address[i], HEX);
        Serial.print(i < 2 ? ":" : "\n");
    }

    Serial.println("----------------------");
}