void startupDisplay()
{
  Serial.print("Starting Display...");
  //AW Code
  Serial.println(" Done");
}

void displayBargraph(uint8_t location, uint8_t length)
{
  
}

void showBattery()
{
  if(millis()-last_packet < 1000)
  {
    if(telemetry.foil_bat != 0xFF)
    {
      if(telemetry.foil_bat < 10)
      {
        if(tlm_last_update != 0)
        {
          tlm_last_update = 0;
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
        }
        else
        {
          if(sq_graph)
          {
            sq_graph = 0;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
          }
          else
          {
            sq_graph = 1;
            aw.analogWrite(AP_LL1, 0);
            aw.analogWrite(AP_LR1, 0);
          }
        }
      }
      else
      {
        if(telemetry.foil_bat >= 80)
        {
          if(tlm_last_update != 5)
          {
            tlm_last_update = 5;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL5, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR5, LED_BRIGHTNESS);
          }
        }
        else if(telemetry.foil_bat >= 60)
        {
          if(tlm_last_update != 4)
          {
            tlm_last_update = 4;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL5, 0);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR5, 0);
          }
        }
        else if(telemetry.foil_bat >= 40)
        {
          if(tlm_last_update != 3)
          {
            tlm_last_update = 3;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL4, 0);
            aw.analogWrite(AP_LL5, 0);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR4, 0);
            aw.analogWrite(AP_LR5, 0);
          }
        }
        else if(telemetry.foil_bat >= 20)
        {
          if(tlm_last_update != 2)
          {
            tlm_last_update = 2;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL3, 0);
            aw.analogWrite(AP_LL4, 0);
            aw.analogWrite(AP_LL5, 0);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR3, 0);
            aw.analogWrite(AP_LR4, 0);
            aw.analogWrite(AP_LR5, 0);
          }
        }
        else
        {
          if(tlm_last_update != 1)
          {
            tlm_last_update = 1;
            aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LL2, 0);
            aw.analogWrite(AP_LL3, 0);
            aw.analogWrite(AP_LL4, 0);
            aw.analogWrite(AP_LL5, 0);
            aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
            aw.analogWrite(AP_LR2, 0);
            aw.analogWrite(AP_LR3, 0);
            aw.analogWrite(AP_LR4, 0);
            aw.analogWrite(AP_LR5, 0);
          }
        }
      }
    }
    else
    {
      if(tlm_last_update != 10)
      {
        tlm_last_update = 10;
        aw.analogWrite(AP_LL1, 0);
        aw.analogWrite(AP_LL2, 0);
        aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
        aw.analogWrite(AP_LL4, 0);
        aw.analogWrite(AP_LL5, 0);
        aw.analogWrite(AP_LR1, 0);
        aw.analogWrite(AP_LR2, 0);
        aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
        aw.analogWrite(AP_LR4, 0);
        aw.analogWrite(AP_LR5, 0);
      }
    }
  }
  else
  {
    if(sq_graph)
    {
      sq_graph = 0;
      aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
      aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
      aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
      aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
      aw.analogWrite(AP_LR5, LED_BRIGHTNESS);

      aw.analogWrite(AP_LL1, 0);
      aw.analogWrite(AP_LL2, 0);
      aw.analogWrite(AP_LL3, 0);
      aw.analogWrite(AP_LL4, 0);
      aw.analogWrite(AP_LL5, 0);
    }
    else
    {
      sq_graph = 1;
      aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL5, LED_BRIGHTNESS);

      aw.analogWrite(AP_LR1, 0);
      aw.analogWrite(AP_LR2, 0);
      aw.analogWrite(AP_LR3, 0);
      aw.analogWrite(AP_LR4, 0);
      aw.analogWrite(AP_LR5, 0);
    }
  }
}

void showNewGear()
{
  tlm_last_update = 0xFF;
  for(int i = 0; i < 5; i++)
  {
    if(gear >= 5)
    {
      aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
    }
    if(gear >= 6)
    {
      aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
    }
    if(gear >= 7)
    {
      aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
    }
    if(gear >= 8)
    {
      aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
    }
    if(gear >= 9)
    {
      aw.analogWrite(AP_LR5, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL5, LED_BRIGHTNESS);
    }
    delay(100);
    aw.analogWrite(AP_LR1, 0);
    aw.analogWrite(AP_LL1, 0);
    aw.analogWrite(AP_LR2, 0);
    aw.analogWrite(AP_LL2, 0);
    aw.analogWrite(AP_LR3, 0);
    aw.analogWrite(AP_LL3, 0);
    aw.analogWrite(AP_LR4, 0);
    aw.analogWrite(AP_LL4, 0);
    aw.analogWrite(AP_LR5, 0);
    aw.analogWrite(AP_LL5, 0);
    delay(100);
  }
}

void displayError(int err)
{
  
}

void bootAnimation()
{
  for(int i = 0; i < 4; i++)
  {
    aw.analogWrite(AP_LR1, 0);
    aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
    
    delay(100);
    aw.analogWrite(AP_LL1, 0);
    aw.analogWrite(AP_LL2, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LL2, 0);
    aw.analogWrite(AP_LL3, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LL3, 0);
    aw.analogWrite(AP_LL4, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LL4, 0);
    aw.analogWrite(AP_LL5, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LL5, 0);
    aw.analogWrite(AP_LR5, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LR5, 0);
    aw.analogWrite(AP_LR4, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LR4, 0);
    aw.analogWrite(AP_LR3, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LR3, 0);
    aw.analogWrite(AP_LR2, LED_BRIGHTNESS);

    delay(100);
    aw.analogWrite(AP_LR2, 0);
    aw.analogWrite(AP_LR1, LED_BRIGHTNESS);

    delay(100);
  }
  aw.analogWrite(AP_LR1, 0);

  uint8_t temp_volt = uint8_t(int_bat_volt*10);

  for(int i = 0; i < 5; i++)
  {
    if(temp_volt > 36)
    {
      aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
    }
    if(temp_volt > 37)
    {
      aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
    }
    if(temp_volt > 38)
    {
      aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
    }
    if(temp_volt > 39)
    {
      aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
    }
    if(temp_volt > 40)
    {
      aw.analogWrite(AP_LR5, LED_BRIGHTNESS);
      aw.analogWrite(AP_LL5, LED_BRIGHTNESS);
    }
    delay(100);
    aw.analogWrite(AP_LR1, 0);
    aw.analogWrite(AP_LL1, 0);
    aw.analogWrite(AP_LR2, 0);
    aw.analogWrite(AP_LL2, 0);
    aw.analogWrite(AP_LR3, 0);
    aw.analogWrite(AP_LL3, 0);
    aw.analogWrite(AP_LR4, 0);
    aw.analogWrite(AP_LL4, 0);
    aw.analogWrite(AP_LR5, 0);
    aw.analogWrite(AP_LL5, 0);
    delay(100);
  }
}


uint8_t arrowPos = 0;
void advanceArrow()
{
  if(arrowPos == 0)
  {
    arrowPos++;
    aw.analogWrite(AP_LR1, LED_BRIGHTNESS);
    aw.analogWrite(AP_LL1, LED_BRIGHTNESS);
    aw.analogWrite(AP_LR2, 0);
    aw.analogWrite(AP_LL2, 0);
    aw.analogWrite(AP_LR3, 0);
    aw.analogWrite(AP_LL3, 0);
    aw.analogWrite(AP_LR4, 0);
    aw.analogWrite(AP_LL4, 0);
    aw.analogWrite(AP_LR5, 0);
    aw.analogWrite(AP_LL5, 0);
  }
  else if(arrowPos == 1)
  {
    arrowPos++;
    aw.analogWrite(AP_LR2, LED_BRIGHTNESS);
    aw.analogWrite(AP_LL2, LED_BRIGHTNESS);
  }
  else if(arrowPos == 2)
  {
    arrowPos++;
    aw.analogWrite(AP_LR3, LED_BRIGHTNESS);
    aw.analogWrite(AP_LL3, LED_BRIGHTNESS);
  }
  else if(arrowPos == 3)
  {
    arrowPos++;
    aw.analogWrite(AP_LR4, LED_BRIGHTNESS);
    aw.analogWrite(AP_LL4, LED_BRIGHTNESS);
  }
  else
  {
    arrowPos = 0;
    aw.analogWrite(AP_LR5, LED_BRIGHTNESS);
    aw.analogWrite(AP_LL5, LED_BRIGHTNESS);
  }
}

void updateBargraphs(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(200);

  while (1) 
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}