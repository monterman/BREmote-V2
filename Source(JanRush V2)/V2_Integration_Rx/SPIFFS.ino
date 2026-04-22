// RX-specific SPIFFS definitions.
// Shared engine is in ../Common/SPIFFSEngine.h (included via BREmote_V2_Rx.h).

void spiffsFormatNotify(bool starting)
{
  if(starting)
  {
    aw.digitalWrite(AP_L_AUX, LOW);
  }
  else
  {
    aw.digitalWrite(AP_L_AUX, HIGH);
  }
}

void spiffsErrorHalt(int type)
{
  if(type == 1) while(1) blinkErr(3, AP_L_AUX);
  while(1) blinkErr(4, AP_L_AUX);
}

// ===== Battery Calibration (RX-only) =====

void getBCFromSPIFFS()
{
  Serial.println("Getting bat conf from SPIFFS...");
  if (readBCFromSPIFFS())
  {

  }
  else
  {
    Serial.println("No batconf in SPIFFS, writing default...");
    //Generate Device Address

    String data = "ANna2dfV1NLQz83LycjGxMPBv768uri3tbOysK6sq6mnpqSioZ+dm5qYlpWTkZCOjIqJh4WEgoB/fXt5eHZ0c3FvbmxqaGdlY2JgXl1bWVdWVFJRT01LSkhGRUNBQD48Ojk3NTQy";
    uint8_t* encodedData = new uint8_t[data.length()];
    // Call the function to fill the encodedData array
    for (size_t i = 0; i < data.length(); i++) {
      encodedData[i] = data[i];  // Convert each character to uint8_t
    }

    // Save to SPIFFS
    File file = SPIFFS.open(BC_FILE_PATH, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        delete[] encodedData;
        return;
    }
    file.write(encodedData, data.length());
    file.close();
    Serial.println("BC saved to SPIFFS as Base64");
    delete[] encodedData;

    if (!readBCFromSPIFFS())
    {
      Serial.println("Error writing bat conf!");
      while(1) blinkErr(4, AP_L_AUX);
    }
  }
  Serial.println("... Done");
}

bool readBCFromSPIFFS() {
    if (!SPIFFS.exists(BC_FILE_PATH)) {
        Serial.println("File does not exist");
        return false;
    }

    // Read file from SPIFFS
    File file = SPIFFS.open(BC_FILE_PATH, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return false;
    }

    String encodedString = file.readString();
    Serial.println("Encoded Data Read: " + encodedString);
    file.close();

    // Decode Base64
    size_t decodedLen = 0;
    mbedtls_base64_decode(NULL, 0, &decodedLen, (const uint8_t*)encodedString.c_str(), encodedString.length());
    uint8_t* decodedData = new uint8_t[decodedLen];
    if (mbedtls_base64_decode(decodedData, decodedLen, &decodedLen, (const uint8_t*)encodedString.c_str(), encodedString.length()) != 0) {
        Serial.println("Base64 decoding failed");
        delete[] decodedData;
        return false;
    }

    noload_offset = ((float)decodedData[0]) / 500.0;
    memcpy(bc_arr, &decodedData[1], 101 * sizeof(decodedData[0]));

    Serial.print("batcal: noload_offset: ");
    Serial.print(noload_offset);
    Serial.print("V, 100% rem. @ ");
    Serial.print((float)bc_arr[0] / 100.0 + 2.0);
    Serial.print("V, 0% rem. @ ");
    Serial.print((float)bc_arr[100] / 100.0 + 2.0);
    Serial.println("V.");

    delete[] decodedData;
    Serial.println("Struct successfully read from SPIFFS");
    return true;
}

void deleteBCFromSPIFFS() {
    if (SPIFFS.remove(BC_FILE_PATH)) {
        Serial.println("File deleted successfully");
    } else {
        Serial.println("Failed to delete file");
    }
}
