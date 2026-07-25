// V2.5-Evo - 2026-07-25 - A1: getLinkQuality() RSSI floor -100 -> -118 dBm (real SF6/BW250 sensitivity). Fixes the signal bar saturating to 0 — and the false weak-signal haptic buzz — while the link still had ~18 dB of margin. Score range (0-10), weighting, TX 0-20 sum and 0-5 bar mapping all unchanged. SHARED HEADER: TX and RX must BOTH be reflashed or the two sides score the same link differently. No confStruct change; no SW_VERSION bump on either side.
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
    // V2.5-Evo - 2026-06-07 - Removed Ludwig's "preset 3" placeholder (no 3rd band or
    // radio type ever shipped — only EU868 / US915 exist) and its radioErrorHalt() boot
    // brick. Any out-of-range value (legacy/corrupt config; config + web UI already clamp
    // to 1-2) now defaults to EU868 instead of halting, so a stale radio_preset can never
    // brick the device again.
    Serial.print(" [radio_preset out of range -> default EU868]");
    state = radio.begin(869.525, 250.0, 6, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, usrConf.rf_power, 8, 1.8, false);
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
  // Normalize RSSI to a 0-10 score across the range this line actually maps:
  //   -118 dBm (floor, score 0) .. -50 dBm (ceiling, score 10).
  //
  // V2.5-Evo - 2026-07-25 - A1 fix. WHAT WAS WRONG: the floor was -100 dBm, and the old comment
  // here claimed "-130 dBm to -50 dBm" — a range the code never used. At the radio settings this
  // firmware runs (SF6, BW 250 kHz — see initRadioHardware above) the SX1262's real demodulation
  // sensitivity is about -118/-119 dBm. A link sitting at -100 dBm therefore still has roughly
  // 18 dB of usable margin, but the old floor scored it 0. WHY THAT MATTERED: the TX adds its own
  // and the RX's score and maps the 0-20 sum onto a 0-5 bar (Display.ino), so the bar collapsed to
  // one segment on a perfectly healthy link, and the one-segment condition is exactly what fires
  // the weak-signal vibration warning (TX System.ino) — a false alarm mid-session.
  // WHAT THE FIX DOES: moves the floor to the real sensitivity, so score 0 now means "at the edge
  // of the radio's ability to hear" instead of "18 dB of margin left". Nothing else is rescaled:
  // the output stays 0-10, the 0.7/0.3 RSSI/SNR weighting is untouched, and the TX-side 0-20 sum
  // and 0-5 bar mapping are unchanged.
  // NOTE: this header is shared by TX and RX — both must be reflashed together, otherwise the two
  // sides score the same link on different scales and their contributions to the sum disagree.
  int rssiScore = constrain(map(rssi, -118, -50, 0, 10), 0, 10);

  // Normalize SNR: Typical range (-20 dB to +10 dB)
  int snrScore = constrain(map(snr, -10, 10, 0, 10), 0, 10);

  // Weighted average (adjust weights as needed)
  float combinedScore = (0.7 * rssiScore) + (0.3 * snrScore);

  // Convert to integer and ensure it's in range 0-10
  return constrain(round(combinedScore), 0, 10);
}

#endif // RADIO_COMMON_H
