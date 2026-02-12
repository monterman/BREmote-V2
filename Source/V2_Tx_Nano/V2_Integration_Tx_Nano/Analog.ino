void startupADS()
{
  Serial.print("Starting ADS1115...");
  if(!ads.begin(ADS1115_ADDRESS))
  {
    Serial.println(" Failed");
    while (true) delay(100);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  Serial.println(" Done");
}

void startupAW()
{
  Serial.print("Starting AW9532...");
  
  if (! aw.begin(0x5B)) {
    Serial.println("AW9523 not found!");
    while (1) delay(10);  // halt forever
  }

  aw.pinMode(AP_LL1, AW9523_LED_MODE);
  aw.pinMode(AP_LL2, AW9523_LED_MODE);
  aw.pinMode(AP_LL3, AW9523_LED_MODE);
  aw.pinMode(AP_LL4, AW9523_LED_MODE);
  aw.pinMode(AP_LL5, AW9523_LED_MODE);
  aw.pinMode(AP_LR1, AW9523_LED_MODE);
  aw.pinMode(AP_LR2, AW9523_LED_MODE);
  aw.pinMode(AP_LR3, AW9523_LED_MODE);
  aw.pinMode(AP_LR4, AW9523_LED_MODE);
  aw.pinMode(AP_LR5, AW9523_LED_MODE);


  aw.analogWrite(AP_LL1, 0);
  aw.analogWrite(AP_LL2, 0);
  aw.analogWrite(AP_LL3, 0);
  aw.analogWrite(AP_LL4, 0);
  aw.analogWrite(AP_LL5, 0);
  aw.analogWrite(AP_LR1, 0);
  aw.analogWrite(AP_LR2, 0);
  aw.analogWrite(AP_LR3, 0);
  aw.analogWrite(AP_LR4, 0);
  aw.analogWrite(AP_LR5, 0);


  Serial.println(" Done");
}

void measBufCalc(void *parameter) 
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (1) 
  {
    measureAndBuffer();
    calcFilter();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
//3ms
void measureAndBuffer()
{
  if (ads.conversionComplete())
  {
    if(last_channel == 0)
    {
      //Serial.print("Read Ch0, pos ");
      //Serial.println(filter_count);
      thr_raw[filter_count] = ads.getLastConversionResults();
      last_channel ++;
    }
    else if(last_channel == 1)
    {
      //Serial.print("Read Ch1, pos ");
      //Serial.println(filter_count);
      tog_raw[filter_count] = ads.getLastConversionResults();
      filter_count++;
      last_channel = 0;
      if(filter_count >= BUFFSZ)
      {
        filter_count = 0;
        last_channel = 3;
      }
    }
    else if(last_channel == 3)
    {
      //Serial.print("Read Ch3, pos ");
      //Serial.println(bat_filter_count);
      if(!mot_active) intbat_raw[bat_filter_count] = ads.getLastConversionResults();
      last_channel = 0;
      bat_filter_count++;
      if(bat_filter_count >= BUFFSZ)
      {
        bat_filter_count = 0;
      }
    }
    else
    {
      last_channel = 0;
    }
  }
  ads.startADCReading(MUX_BY_CHANNEL[last_channel],false);
}