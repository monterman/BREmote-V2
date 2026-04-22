#ifndef RADIO_COMMON_H
#define RADIO_COMMON_H

// Shared radio init and helpers for BREmote V2 TX and RX.
// Requirements before #include:
//   - <RadioLib.h> included
//   - <SPI.h> included
//   - SX1262 radio global declared
//   - confStruct with rf_power, radio_preset fields
//   - usrConf global declared
//   - P_SPI_SCK, P_SPI_MISO, P_SPI_MOSI pin defines
//
// Each side must define:
//   void radioErrorHalt(int type);
//     type 1 = invalid transmit power
//     type 2 = unsupported radio preset
//     type 3 = radio init failed
//   void radioInitSuccess();

// Forward declarations — defined per-side in Radio.ino / main .ino.
void radioErrorHalt(int type);
void radioInitSuccess();
extern SX1262 radio;

static int initRadioHardware()
{
  Serial.print("Starting Radio...");

  SPI.begin(P_SPI_SCK, P_SPI_MISO, P_SPI_MOSI);

  if(usrConf.rf_power < -9 || usrConf.rf_power > 22)
  {
    Serial.println("Error, invalid transmit power");
    radioErrorHalt(1);
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
  else
  {
    Serial.println("Error, unsupported HF setting");
    radioErrorHalt(2);
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
    radioInitSuccess();
  }
  else
  {
    Serial.print(" Failed, code: ");
    Serial.println(state);
    radioErrorHalt(3);
  }

  return state;
}

static int getLinkQuality(float rssi, float snr)
{
  // Normalize RSSI: Expected range (-130 dBm to -50 dBm)
  int rssiScore = constrain(map(rssi, -100, -50, 0, 10), 0, 10);

  // Normalize SNR: Typical range (-20 dB to +10 dB)
  int snrScore = constrain(map(snr, -10, 10, 0, 10), 0, 10);

  // Weighted average (adjust weights as needed)
  float combinedScore = (0.7 * rssiScore) + (0.3 * snrScore);

  // Convert to integer and ensure it's in range 0-10
  return constrain(round(combinedScore), 0, 10);
}

#endif // RADIO_COMMON_H
