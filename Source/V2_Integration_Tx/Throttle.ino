// V2.5-Evo - 2026-04-25 - P7: calcFinalThrottle() applies rtm_thr_cap_tx ramp
// Centralized throttle calculation module.
// Handles gear mode (0), no-gear mode (1), and dynamic cap mode (2).

uint8_t calcFinalThrottle()
{
  uint8_t shaped = expoThrCurve(thr_scaled);
  uint8_t result;
  switch(usrConf.throttle_mode)
  {
    case 1: // No gears — full power, fixed at max gear
      result = shaped;
      break;
    case 2: // Dynamic cap
      result = (uint8_t)((uint16_t)shaped * max_power_cap / 100);
      break;
    case 0: // Gears
    default:
      result = (uint8_t)((uint16_t)shaped * (gear + 1) / usrConf.max_gears);
      break;
  }

  // V2.5-Evo - 2026-04-25 - P7: Apply RTM throttle ramp cap.
  // rtm_thr_cap_tx is 255 when RTM is not active (no effect on normal operation).
  // When RTM is ACTIVE, ramps from rtm_throttle_start_pct to rtm_throttle_max_pct.
  // This enforces the creator safety rule: RTM can only subtract from user throttle.
  if (result > rtm_thr_cap_tx) result = rtm_thr_cap_tx;

  return result;
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
