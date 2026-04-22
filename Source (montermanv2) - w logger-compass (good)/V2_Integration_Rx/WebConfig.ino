// RX-specific web config definitions.
// Shared engine is in ../Common/WebConfigEngine.h (included via BREmote_V2_Rx.h).

#ifdef WIFI_ENABLED

const char* WEB_CFG_AP_SSID = "BREmoteV2-RX-WebConfig";
const char* WEB_CFG_SHUTDOWN_REASON = "rx_connected";

void webCfgResetCalibration(confStruct& conf)
{
  conf.ubat_cal = defaultConf.ubat_cal;
  conf.ubat_offset = defaultConf.ubat_offset;
}

void webCfgNotifyRxConnected()
{
  web_cfg_should_shutdown = true;
}

#endif // WIFI_ENABLED
