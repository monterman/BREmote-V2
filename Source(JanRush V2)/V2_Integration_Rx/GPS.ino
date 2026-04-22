void configureGPS() {
  // UBX-CFG-PRT packet for 115200 baud (UART1, 8N1, UBX+NMEA)
  // Checksum is pre-calculated here for reliability
  byte setBaud[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 
    0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
  };

  // UBX-CFG-RATE for 5Hz (200ms)
  byte setRate[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 
    0x01, 0x00, 0xDE, 0x6A
  };
  setUartMux(1);
  // 1. Initial connection at default GPS speed
  Serial.println("GPS: Connecting at 9600...");
  Serial1.begin(9600, SERIAL_8N1, P_U1_RX, P_U1_TX);
  delay(200);

  // 2. Send Baud Change Command
  Serial1.write(setBaud, sizeof(setBaud));
  
  // CRITICAL: flush() only clears the buffer, hardware needs time to send
  Serial1.flush(); 
  delay(50); // Give the UART hardware ~50ms to finish sending the packet
  
  // 3. Switch ESP32 to the new speed
  Serial1.end();
  delay(100);
  Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  Serial.println("GPS: Baud switched to 115200");

  // 4. Send Rate Command at the new speed
  delay(100);
  Serial1.write(setRate, sizeof(setRate));
  Serial1.flush();
  Serial.println("GPS: Config Complete (115200 baud, 5Hz)");
}

// Task for GPS reading
void getGPSLoop()
{
  setUartMux(1);
  vTaskDelay(pdMS_TO_TICKS(10));  
  //Serial1.end();
  //Serial1.begin(115200, SERIAL_8N1, P_U1_RX, P_U1_TX);
  //while(!Serial1) vTaskDelay(pdMS_TO_TICKS(10));
  
  // Flush the serial buffer to get fresh data
  Serial1.flush();
  
  // Reset buffer for new reading
  bool newData = false;
  unsigned long startTime = millis();
  
  while (millis() - startTime < 300) {
    if (Serial1.available())
    {
      char c = Serial1.read();
      // Feed each character to TinyGPS++ object
      if (gps.encode(c)) {
        newData = true;
      }
    }
    else
    {
      // No data available, yield to other tasks
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }

  if (!newData)
  {
    telemetry.foil_speed = 0xFF;
  }
  else if(!gps.speed.isUpdated())
  {
    telemetry.foil_speed = 99;
  }
  else
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  }
/*
  // If we received valid GPS data
  if (newData && gps.speed.isUpdated()) 
  {
    telemetry.foil_speed = (uint8_t)gps.speed.kmph();
  } 
  else 
  {
    //telemetry.foil_speed = 0xFF;
  }*/
}

// Function to print satellite information
void printSatelliteInfo() {
  Serial.println("----- GPS Satellite Status -----");
  Serial.print("Satellites in view: ");
  Serial.println(gps.satellites.value());
  
  Serial.print("HDOP (Horizontal Dilution of Precision): ");
  if (gps.hdop.isValid()) {
    Serial.print(gps.hdop.value());
    Serial.println(" (Lower is better, <1 Excellent, 1-2 Good, 2-5 Moderate, 5-10 Fair, >10 Poor)");
  } else {
    Serial.println("Invalid");
  }
  
  Serial.print("Location validity: ");
  Serial.println(gps.location.isValid() ? "Valid" : "Invalid");
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.println(" meters");
    } else {
      Serial.println("Invalid");
    }
  }
  
  Serial.print("Date/Time validity: ");
  Serial.println(gps.date.isValid() && gps.time.isValid() ? "Valid" : "Invalid");
  
  if (gps.date.isValid() && gps.time.isValid()) {
    char dateTime[30];
    sprintf(dateTime, "%04d-%02d-%02d %02d:%02d:%02d UTC", 
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    Serial.print("Date/Time: ");
    Serial.println(dateTime);
  }
  
  Serial.print("Course validity: ");
  Serial.println(gps.course.isValid() ? "Valid" : "Invalid");
  
  if (gps.course.isValid()) {
    Serial.print("Course: ");
    Serial.print(gps.course.deg());
    Serial.println(" degrees");
  }
  
  Serial.print("Chars processed: ");
  Serial.println(gps.charsProcessed());
  Serial.print("Sentences with fix: ");
  Serial.println(gps.sentencesWithFix());
  Serial.print("Failed checksum: ");
  Serial.println(gps.failedChecksum());
  
  Serial.println("-------------------------------");
}