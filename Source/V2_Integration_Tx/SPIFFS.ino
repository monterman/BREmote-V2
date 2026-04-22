// TX-specific SPIFFS definitions.
// Shared engine is in ../Common/SPIFFSEngine.h (included via BREmote_V2_Tx.h).

void spiffsFormatNotify(bool starting)
{
  if(starting)
  {
    displayDigits(LET_I, 5);
    updateDisplay();
  }
}

void spiffsErrorHalt(int type)
{
  if(type == 1) while(1) scroll4Digits(LET_E, 5, LET_P, 3, 200);
  while(1) scroll4Digits(LET_E, 5, LET_P, 4, 200);
}
