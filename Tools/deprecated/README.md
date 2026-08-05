# Deprecated tools — kept for reference, do not use

These no longer match the current firmware. They are here so old links and old forum posts
still resolve, not because they work.

**Use this instead:**

```
docs/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html
```

Live at <https://monterman.github.io/BREmote-V2/BREmote_V2.5-Evo_Web_Serial_Config_Tool.html>.
It is the only config tool kept in sync with `WebUiEmbedded.h` on both boards.

---

## `BREmote_V2.5-Evo_Config_Studio_AFM_v4_DEPRECATED.html`

Superseded by the Web Serial tool. Its field definitions are far enough behind the current
`confStruct` that it no longer works against this firmware's settings.

## `config_converter_DEPRECATED.html`

Converted a stored config between struct-layout generations, labelled V1 / V2 / V3 in its
dropdowns. *(Those labels refer to **config-format generations**, not to firmware versions —
the firmware is V2.5-Evo and there was never a "V3" release.)*

Retired because the job it did no longer pays. Converting a config forward from Ludwig's branch
lands you in a struct with dozens of fields the old config never had, so you would be hand-tuning
most of it anyway — and this firmware's baked defaults are a better starting point than a
partially converted config.

**If you are coming from an older branch: flash, then set your values from defaults.** It is less
work than converting, and you will not inherit a half-populated config that looks configured but
is not.
