void startupADS()
{
  Serial.print("Starting ADS1115...");
  if(!ads.begin(ADS1115_ADDRESS))
  {
    Serial.println(" Failed");
    while (true) delay(100);
  }
  ads.setGain(GAIN_ONE);
  Serial.println(" Done");
}

void setHallActivityEnabled(bool enabled)
{
  hall_activity_enabled = enabled;

  if(enabled)
  {
    filter_count = 0;
    bat_filter_count = 0;
    last_channel = 0;
    ads.startADCReading(MUX_BY_CHANNEL[last_channel],false);
    return;
  }

  thr_scaled = 0;
  tog_scaled = 127;
  steer_scaled = 127;
  tog_input = 0;
}

bool isHallActivityEnabled()
{
  return hall_activity_enabled;
}

void measBufCalc(void *parameter) 
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  while (1) 
  {
    if(isHallActivityEnabled())
    {
      measureAndBuffer();
      calcFilter();
    }
    else
    {
      thr_scaled = 0;
      tog_scaled = 127;
      steer_scaled = 127;
      tog_input = 0;
    }
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
