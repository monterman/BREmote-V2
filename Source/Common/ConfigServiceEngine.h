#ifndef CONFIG_SERVICE_ENGINE_H
#define CONFIG_SERVICE_ENGINE_H

// Shared config engine for BREmote V2 TX and RX.
// Requirements before #include:
//   - confStruct type defined
//   - usrConf global variable declared
//   - SW_VERSION constant defined
//
// Each side must define (in its own ConfigService.ino):
//   static const CfgFieldSpec kCfgFields[];
//   static const size_t kCfgFieldCount;
//   static bool cfgValidateCrossField(confStruct&, String&);

#include <stddef.h>

enum CfgFieldType
{
  CFG_U16,
  CFG_I16,
  CFG_FLOAT,
  CFG_ADDR3,
  CFG_STR8
};

struct CfgFieldSpec
{
  const char* key;
  CfgFieldType type;
  size_t offset;
  bool writable;
  bool radioReinitRequired;
  bool hasRange;
  float minValue;
  float maxValue;
  uint8_t floatPrecision;
  bool mustEqualSwVersion;
};

// Forward declarations — defined per-side in ConfigService.ino.
// Uses extern since the definitions appear later in the same translation unit
// (Arduino concatenates all .ino files into one .cpp).
extern const CfgFieldSpec kCfgFields[];
extern const size_t kCfgFieldCount;
bool cfgValidateCrossField(confStruct &candidate, String &err);

// ===== Parsing Helpers =====

static bool cfgParseIntStrict(const String& text, long &out)
{
  String s = text;
  s.trim();
  if (s.length() == 0) return false;

  char *endPtr = NULL;
  long value = strtol(s.c_str(), &endPtr, 10);
  if (endPtr == s.c_str() || *endPtr != '\0') return false;

  out = value;
  return true;
}

static bool cfgParseFloatStrict(const String& text, float &out)
{
  String s = text;
  s.trim();
  if (s.length() == 0) return false;

  char *endPtr = NULL;
  float value = strtof(s.c_str(), &endPtr);
  if (endPtr == s.c_str() || *endPtr != '\0') return false;
  if (isnan(value) || isinf(value)) return false;

  out = value;
  return true;
}

static bool cfgParseUInt16Strict(const String& text, uint16_t &out)
{
  long v = 0;
  if (!cfgParseIntStrict(text, v)) return false;
  if (v < 0 || v > 65535) return false;
  out = (uint16_t)v;
  return true;
}

static bool cfgParseByteToken(const String& tokenIn, uint8_t &out, bool forceHex)
{
  String token = tokenIn;
  token.trim();
  if (token.length() == 0) return false;

  bool hexLike = false;
  for (int i = 0; i < token.length(); i++)
  {
    const char c = token[i];
    if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))
    {
      hexLike = true;
      break;
    }
  }

  int base = 10;
  if (forceHex || token.startsWith("0x") || token.startsWith("0X") || hexLike) base = 16;

  char *endPtr = NULL;
  long value = strtol(token.c_str(), &endPtr, base);
  if (endPtr == token.c_str() || *endPtr != '\0') return false;
  if (value < 0 || value > 255) return false;

  out = (uint8_t)value;
  return true;
}

static bool cfgParseAddress3(const String& textIn, uint8_t out[3])
{
  String text = textIn;
  text.trim();
  if (text.length() == 0) return false;
  const bool forceHex = (text.indexOf(':') >= 0 || text.indexOf('-') >= 0);

  text.replace(':', ',');
  text.replace('-', ',');
  text.replace(';', ',');

  int start = 0;
  int idx = 0;
  while (start <= text.length())
  {
    int sep = text.indexOf(',', start);
    if (sep < 0) sep = text.length();
    String token = text.substring(start, sep);
    token.trim();

    if (token.length() > 0)
    {
      if (idx >= 3) return false;
      if (!cfgParseByteToken(token, out[idx], forceHex)) return false;
      idx++;
    }

    if (sep >= text.length()) break;
    start = sep + 1;
  }

  return idx == 3;
}

static String cfgFormatAddress3(const uint8_t addr[3])
{
  char buf[9];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X", addr[0], addr[1], addr[2]);
  return String(buf);
}

// ===== Key Lookup =====

static String cfgNormalizeKey(const String& keyIn)
{
  String key = keyIn;
  key.trim();
  key.toLowerCase();
  return key;
}

static const CfgFieldSpec* cfgFindFieldByKey(const String& keyIn)
{
  const String key = cfgNormalizeKey(keyIn);
  for (size_t i = 0; i < kCfgFieldCount; i++)
  {
    if (key == kCfgFields[i].key) return &kCfgFields[i];
  }
  return NULL;
}

// ===== Field Read/Write =====

static bool cfgApplyFieldValue(confStruct &target, const CfgFieldSpec& spec, const String& valueIn, String &err)
{
  if (!spec.writable)
  {
    err = "ERR_READ_ONLY:" + String(spec.key);
    return false;
  }

  uint8_t* base = reinterpret_cast<uint8_t*>(&target);
  uint8_t* ptr = base + spec.offset;

  if (spec.type == CFG_ADDR3)
  {
    uint8_t parsed[3];
    if (!cfgParseAddress3(valueIn, parsed))
    {
      err = "ERR_BAD_VALUE:" + String(spec.key);
      return false;
    }
    uint8_t* addr = reinterpret_cast<uint8_t*>(ptr);
    addr[0] = parsed[0];
    addr[1] = parsed[1];
    addr[2] = parsed[2];
    return true;
  }

  if (spec.type == CFG_STR8)
  {
    String sv = valueIn;
    sv.trim();
    if (sv.length() != 8)
    {
      err = "ERR_BAD_VALUE:" + String(spec.key) + " (must be exactly 8 chars)";
      return false;
    }
    for (int ci = 0; ci < 8; ci++)
    {
      if (sv[ci] < 0x20 || sv[ci] > 0x7E)
      {
        err = "ERR_BAD_VALUE:" + String(spec.key) + " (non-printable char)";
        return false;
      }
    }
    memcpy(ptr, sv.c_str(), 8);
    return true;
  }

  if (spec.type == CFG_FLOAT)
  {
    float fv = 0.0f;
    if (!cfgParseFloatStrict(valueIn, fv))
    {
      err = "ERR_BAD_VALUE:" + String(spec.key);
      return false;
    }
    if (spec.hasRange && (fv < spec.minValue || fv > spec.maxValue))
    {
      err = "ERR_RANGE:" + String(spec.key);
      return false;
    }
    *reinterpret_cast<float*>(ptr) = fv;
    return true;
  }

  if (spec.type == CFG_U16)
  {
    uint16_t u16 = 0;
    if (!cfgParseUInt16Strict(valueIn, u16))
    {
      err = "ERR_BAD_VALUE:" + String(spec.key);
      return false;
    }

    if (spec.mustEqualSwVersion && u16 != SW_VERSION)
    {
      err = "ERR_RANGE:" + String(spec.key);
      return false;
    }
    if (spec.hasRange && ((float)u16 < spec.minValue || (float)u16 > spec.maxValue))
    {
      err = "ERR_RANGE:" + String(spec.key);
      return false;
    }

    *reinterpret_cast<uint16_t*>(ptr) = u16;
    return true;
  }

  if (spec.type == CFG_I16)
  {
    long iv = 0;
    if (!cfgParseIntStrict(valueIn, iv))
    {
      err = "ERR_BAD_VALUE:" + String(spec.key);
      return false;
    }

    if (iv < -32768 || iv > 32767)
    {
      err = "ERR_RANGE:" + String(spec.key);
      return false;
    }
    if (spec.hasRange && ((float)iv < spec.minValue || (float)iv > spec.maxValue))
    {
      err = "ERR_RANGE:" + String(spec.key);
      return false;
    }
    *reinterpret_cast<int16_t*>(ptr) = (int16_t)iv;
    return true;
  }

  err = "ERR_BAD_VALUE:" + String(spec.key);
  return false;
}

static bool cfgReadFieldValue(const confStruct &source, const CfgFieldSpec& spec, String &outValue)
{
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&source);
  const uint8_t* ptr = base + spec.offset;

  if (spec.type == CFG_U16)
  {
    outValue = String(*reinterpret_cast<const uint16_t*>(ptr));
    return true;
  }
  if (spec.type == CFG_I16)
  {
    outValue = String(*reinterpret_cast<const int16_t*>(ptr));
    return true;
  }
  if (spec.type == CFG_FLOAT)
  {
    outValue = String(
      (double)(*reinterpret_cast<const float*>(ptr)),
      (unsigned int)spec.floatPrecision
    );
    return true;
  }
  if (spec.type == CFG_ADDR3)
  {
    outValue = cfgFormatAddress3(reinterpret_cast<const uint8_t*>(ptr));
    return true;
  }
  if (spec.type == CFG_STR8)
  {
    char buf[9];
    memcpy(buf, ptr, 8);
    buf[8] = '\0';
    outValue = String(buf);
    return true;
  }
  return false;
}

static bool cfgAppendFieldJson(const confStruct &source, const CfgFieldSpec& spec, String &out)
{
  const uint8_t* base = reinterpret_cast<const uint8_t*>(&source);
  const uint8_t* ptr = base + spec.offset;

  out += "\"";
  out += spec.key;
  out += "\":";

  if (spec.type == CFG_U16)
  {
    out += String(*reinterpret_cast<const uint16_t*>(ptr));
    return true;
  }
  if (spec.type == CFG_I16)
  {
    out += String(*reinterpret_cast<const int16_t*>(ptr));
    return true;
  }
  if (spec.type == CFG_FLOAT)
  {
    out += String(
      (double)(*reinterpret_cast<const float*>(ptr)),
      (unsigned int)spec.floatPrecision
    );
    return true;
  }
  if (spec.type == CFG_ADDR3)
  {
    const uint8_t* addr = reinterpret_cast<const uint8_t*>(ptr);
    out += "\"";
    out += cfgFormatAddress3(addr);
    out += "\"";
    return true;
  }
  if (spec.type == CFG_STR8)
  {
    char buf[9];
    memcpy(buf, ptr, 8);
    buf[8] = '\0';
    out += "\"";
    out += String(buf);
    out += "\"";
    return true;
  }
  return false;
}

// ===== Public API =====

bool cfgGetValueByKey(const String& keyIn, String &outValue, String &err)
{
  const CfgFieldSpec* spec = cfgFindFieldByKey(keyIn);
  if (spec == NULL)
  {
    err = "ERR_UNKNOWN_KEY:" + cfgNormalizeKey(keyIn);
    return false;
  }

  if (!cfgReadFieldValue(usrConf, *spec, outValue))
  {
    err = "ERR_BAD_VALUE:" + String(spec->key);
    return false;
  }

  return true;
}

bool cfgGetAllJson(String &out)
{
  out = "{";
  for (size_t i = 0; i < kCfgFieldCount; i++)
  {
    if (!cfgAppendFieldJson(usrConf, kCfgFields[i], out)) return false;
    if (i + 1 < kCfgFieldCount) out += ",";
  }
  out += "}";
  return true;
}

bool cfgSetValueByKey(const String& key, const String& value, String &err, bool &radioReinitRequired)
{
  const CfgFieldSpec* spec = cfgFindFieldByKey(key);
  if (spec == NULL)
  {
    err = "ERR_UNKNOWN_KEY:" + cfgNormalizeKey(key);
    return false;
  }

  confStruct staged = usrConf;
  if (!cfgApplyFieldValue(staged, *spec, value, err)) return false;
  if (!cfgValidateCrossField(staged, err)) return false;

  radioReinitRequired = spec->radioReinitRequired;
  usrConf = staged;
  return true;
}

bool cfgSetBatch(const String& payload, String &err, bool &radioReinitRequired)
{
  // JSON-only batch format:
  // {"key1":123,"key2":"value","key3":true}
  // Values are converted to plain strings and then validated via cfgApplyFieldValue().
  confStruct staged = usrConf;
  radioReinitRequired = false;

  String s = payload;
  s.trim();
  if (s.length() < 2 || s[0] != '{' || s[s.length() - 1] != '}')
  {
    err = "ERR_BAD_VALUE:batch_json";
    return false;
  }

  int i = 1; // Skip opening '{'
  while (i < s.length() - 1)
  {
    // Skip whitespace and optional commas
    while (i < s.length() - 1 && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n' || s[i] == ',')) i++;
    if (i >= s.length() - 1) break;

    // Parse quoted key
    if (s[i] != '"')
    {
      err = "ERR_BAD_VALUE:batch_json_key";
      return false;
    }
    i++; // past opening quote
    String key = "";
    bool keyClosed = false;
    while (i < s.length() - 1)
    {
      char c = s[i++];
      if (c == '\\')
      {
        if (i >= s.length() - 1)
        {
          err = "ERR_BAD_VALUE:batch_json_key_escape";
          return false;
        }
        key += s[i++];
        continue;
      }
      if (c == '"')
      {
        keyClosed = true;
        break;
      }
      key += c;
    }
    if (!keyClosed)
    {
      err = "ERR_BAD_VALUE:batch_json_key_unclosed";
      return false;
    }
    key.trim();
    if (key.length() == 0)
    {
      err = "ERR_BAD_VALUE:batch_json_key_empty";
      return false;
    }

    // Skip whitespace then expect ':'
    while (i < s.length() - 1 && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
    if (i >= s.length() - 1 || s[i] != ':')
    {
      err = "ERR_BAD_VALUE:batch_json_colon";
      return false;
    }
    i++; // past ':'
    while (i < s.length() - 1 && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
    if (i >= s.length() - 1)
    {
      err = "ERR_BAD_VALUE:batch_json_value";
      return false;
    }

    String value = "";
    if (s[i] == '"')
    {
      // Quoted string value
      i++; // past opening quote
      bool valClosed = false;
      while (i < s.length() - 1)
      {
        char c = s[i++];
        if (c == '\\')
        {
          if (i >= s.length() - 1)
          {
            err = "ERR_BAD_VALUE:batch_json_value_escape";
            return false;
          }
          char e = s[i++];
          if (e == 'n') value += '\n';
          else if (e == 'r') value += '\r';
          else if (e == 't') value += '\t';
          else value += e;
          continue;
        }
        if (c == '"')
        {
          valClosed = true;
          break;
        }
        value += c;
      }
      if (!valClosed)
      {
        err = "ERR_BAD_VALUE:batch_json_value_unclosed";
        return false;
      }
    }
    else
    {
      // Unquoted scalar (number/true/false)
      int startVal = i;
      while (i < s.length() - 1 && s[i] != ',' && s[i] != '}') i++;
      value = s.substring(startVal, i);
      value.trim();
      if (value.length() == 0)
      {
        err = "ERR_BAD_VALUE:batch_json_value_empty";
        return false;
      }
      if (value == "true") value = "1";
      else if (value == "false") value = "0";
      else if (value == "null")
      {
        err = "ERR_BAD_VALUE:batch_json_null";
        return false;
      }
    }

    const CfgFieldSpec* spec = cfgFindFieldByKey(key);
    if (spec == NULL)
    {
      err = "ERR_UNKNOWN_KEY:" + cfgNormalizeKey(key);
      return false;
    }

    if (!cfgApplyFieldValue(staged, *spec, value, err)) return false;
    radioReinitRequired = radioReinitRequired || spec->radioReinitRequired;

    // Skip trailing whitespace between pairs; comma will be consumed by top-of-loop skip.
    while (i < s.length() - 1 && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) i++;
  }

  if (!cfgValidateCrossField(staged, err)) return false;
  usrConf = staged;
  return true;
}

// ===== Table-driven Validator =====

bool validateConfig(const confStruct& conf, String& err)
{
  for (size_t i = 0; i < kCfgFieldCount; i++)
  {
    const CfgFieldSpec& spec = kCfgFields[i];
    const uint8_t* base = reinterpret_cast<const uint8_t*>(&conf);
    const uint8_t* ptr = base + spec.offset;

    if (spec.type == CFG_U16)
    {
      uint16_t value = *(const uint16_t*)ptr;
      if (spec.hasRange)
      {
        if (value < (uint16_t)spec.minValue || value > (uint16_t)spec.maxValue)
        {
          err = "ERR_RANGE:" + String(spec.key) + " (" + String(value) + " not in " + String(spec.minValue) + "-" + String(spec.maxValue) + ")";
          return false;
        }
      }
    }
    else if (spec.type == CFG_I16)
    {
      int16_t value = *(const int16_t*)ptr;
      if (spec.hasRange)
      {
        if (value < (int16_t)spec.minValue || value > (int16_t)spec.maxValue)
        {
          err = "ERR_RANGE:" + String(spec.key) + " (" + String(value) + " not in " + String((int16_t)spec.minValue) + "-" + String((int16_t)spec.maxValue) + ")";
          return false;
        }
      }
    }
    else if (spec.type == CFG_FLOAT)
    {
      float value = *(const float*)ptr;
      if (isnan(value) || isinf(value))
      {
        err = "ERR_NAN:" + String(spec.key);
        return false;
      }
      if (spec.hasRange)
      {
        if (value < spec.minValue || value > spec.maxValue)
        {
          err = "ERR_RANGE:" + String(spec.key) + " (" + String(value) + " not in " + String(spec.minValue) + "-" + String(spec.maxValue) + ")";
          return false;
        }
      }
    }
    // CFG_ADDR3 doesn't need range validation
  }

  return true;
}

#endif // CONFIG_SERVICE_ENGINE_H
