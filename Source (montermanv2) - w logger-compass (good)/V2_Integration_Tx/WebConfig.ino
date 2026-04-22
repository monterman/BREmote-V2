// TX-specific web config definitions.
// Shared engine is in ../Common/WebConfigEngine.h (included via BREmote_V2_Tx.h).

#ifdef WIFI_ENABLED

const char* WEB_CFG_AP_SSID = "BREmoteV2-TX-WebConfig";
const char* WEB_CFG_SHUTDOWN_REASON = "tx_unlocked";

void webCfgResetCalibration(confStruct& conf)
{
  conf.cal_ok = 0;
  conf.cal_offset = 0;
  conf.thr_idle = 0;
  conf.thr_pull = 0;
  conf.tog_left = 0;
  conf.tog_mid = 0;
  conf.tog_right = 0;
}

void webCfgNotifyTxUnlocked()
{
  web_cfg_should_shutdown = true;
}

#endif // WIFI_ENABLED
