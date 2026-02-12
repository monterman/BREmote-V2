#include "BREmote_V2_Tx.h"

SX1262 radio = new Module(P_LORA_NSS, P_LORA_DIO, P_LORA_RST, P_LORA_BUSY);
Adafruit_ADS1115 ads;
Adafruit_AW9523 aw;
Ticker ticksrc;

void setup()
{
  enterSetup();

  Wire.begin(P_I2C_SDA, P_I2C_SCL); //SDA, SCL
  Wire.setClock(400000); // Set to 400 kHz
  startupAW();

  initSPIFFS();
  getConfFromSPIFFS();

  if(usrConf.max_gears <= 0) usrConf.max_gears = 1;

  startupADS();

  checkCharger();

  startupRadio();
  radio.setDio1Action(packetReceived);

  //Runs every 100ms and sends data to Rx, medium prio
  xTaskCreatePinnedToCore(sendData, "Send_Data_100ms", 2048, NULL, 5, &sendDataHandle, 0);
  //Triggered by senData, waits from telem from Rx after sending data to it
  xTaskCreatePinnedToCore(waitForTelemetry, "wait_for_telem_triggered", 2048, NULL, 4, &triggeredWaitForTelemetryHandle, 0);
  //Measure, Buffer and calculate inputs
  xTaskCreatePinnedToCore(measBufCalc, "wait_for_telem_triggered_10ms", 2048, NULL, 6, &measBufCalcHandle, 0);
  //Update bargraphs
  xTaskCreatePinnedToCore(updateBargraphs, "wait_for_telem_triggered_200ms", 2048, NULL, 6, &updateBargraphsHandle, 0);

  //Show boot animation and display internal battery voltage (needed for buffers)
  bootAnimation();
  //Check if calibration is valid, otherwise calibrate
  checkCal();
  //Check if a button combination is input
  checkStartupButtons();
  //Check if system is paired
  checkPairing();

  while(thr_scaled > 10)
  {
    advanceArrow();
    delay(100);
  }
  system_locked = 0;
  usrConf.max_gears = 10;

  if(usrConf.no_gear)
  {
    gear = usrConf.max_gears-1;
  }
  else
  {
    if(usrConf.startgear >= usrConf.max_gears) usrConf.startgear = usrConf.max_gears;
    gear = usrConf.startgear;
    if(gear < 5) gear = 5;
  }

  exitSetup();
}

void loop()
{
  runMenu();
  if(in_menu > 0) in_menu--;

  checkSerial();

  if(remote_error == 0)
  {
    showBattery();
  }
  else
  {
    //Show Error
  }
  
  delay(110);
  
} //End of loop()