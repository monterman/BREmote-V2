// Centralized throttle calculation module.
// Handles gear mode (0), no-gear mode (1), and dynamic cap mode (2).

uint8_t calcFinalThrottle()
{
  uint8_t shaped = expoThrCurve(thr_scaled);
  switch(usrConf.throttle_mode)
  {
    case 1: // No gears — full power, fixed at max gear
      return shaped;
    case 2: // Dynamic cap
      return (uint16_t)shaped * max_power_cap / 100;
    case 0: // Gears
    default:
      return (uint16_t)shaped * (gear + 1) / usrConf.max_gears;
  }
}

void throttleInit()
{
  switch(usrConf.throttle_mode)
  {
    case 1: // No gears — lock gear to max
      gear = usrConf.max_gears - 1;
      break;
    case 2: // Dynamic cap
      max_power_cap = usrConf.dynamic_power_start;
      if(max_power_cap < 10) max_power_cap = 10;
      if(max_power_cap > 100) max_power_cap = 100;
      gear = usrConf.max_gears - 1; // Not used in calc, but keeps display sane
      break;
    case 0: // Gears
    default:
      if(usrConf.startgear >= usrConf.max_gears) usrConf.startgear = usrConf.max_gears - 1;
      gear = usrConf.startgear;
      break;
  }
}

void throttleReset()
{
  throttleInit();
}

void throttleAdjustCap(int direction)
{
  int16_t new_cap = (int16_t)max_power_cap + direction * (int16_t)usrConf.dynamic_power_step;
  if(new_cap < 10) new_cap = 10;
  if(new_cap > 100) new_cap = 100;
  max_power_cap = (uint8_t)new_cap;
}

bool throttleUsesGears()
{
  return usrConf.throttle_mode == 0;
}

bool throttleForceToggleBlock()
{
  return (usrConf.throttle_mode == 1) && usrConf.no_lock;
}

uint8_t throttleGetCapPercent()
{
  return max_power_cap;
}
