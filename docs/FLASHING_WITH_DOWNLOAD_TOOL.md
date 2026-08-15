# Flashing with the Flash Download Tool — the normal way

**This is the recommended way to flash a BREmote.** No Arduino, no compiling, no libraries, no
toolchain. Download a `.bin`, point a small Windows program at it, press START.

It is also the **safest** way: you are flashing the exact image that was built and tested, rather
than a fresh compile that depends on your library versions and board settings being right.

> Compiling from source is the **advanced** path and is documented separately —
> [Flashing the RX (Arduino CLI/IDE)](FLASHING_RX_ARDUINO.md) ·
> [Flashing the TX (Arduino CLI/IDE)](FLASHING_TX_ARDUINO.md). You do not need either one to run
> BREmote. Use them only if you are changing the firmware.

---

## What you need

| | |
|---|---|
| **The tool** | **[Flash Download Tool](https://www.espressif.com/en/support/download/other-tools)** — Espressif's official Windows utility. Also called the *ESP Download Tool* or *ESP32 Download Tool*; all the same program. Free, no install — unzip and run the `.exe`. |
| **Prefer to watch it?** | **[Ludwig's walkthrough, from 40:00 →](https://youtu.be/r6JIZEq3aTU?t=2400)** — he flashes a BREmote with this exact tool. Nothing has changed since; just use **your** `.bin` from the table below. |
| **The firmware** | The current `.bin` for your board — see the table below |
| **A USB cable** | A **data** cable. Charge-only cables are the single most common "my board won't connect" cause. |

---

## Step 1 — Get the right `.bin`

| Board | File | Version |
|---|---|---|
| **TX** (handheld remote) | [`BREmote-TX-SW27-gps-verified.bin`](../Source/V2_Integration_Tx/TX%20firmware/BREmote-TX-SW27-gps-verified.bin) | **SW27** — current |
| **RX** (in the buggy) | [`BREmote-RX-SW34-gps-verified.bin`](../Source/V2_Integration_Rx/RX%20firmware/BREmote-RX-SW34-gps-verified.bin) | **SW34** — current |

Every published build: **[TX firmware →](../Source/V2_Integration_Tx/TX%20firmware/)** ·
**[RX firmware →](../Source/V2_Integration_Rx/RX%20firmware/)**. Each folder has a `README.md`
explaining what each build is. Older ones are kept for rollback and carry known issues — use the
`-gps-verified` files above.

> ### 🚨 Download the RAW file — this is where people go wrong
>
> Click the `.bin` to open it on GitHub, then press the **Download raw file** button (the ⤓ icon at
> the top-right of the file view).
>
> **Do not use your browser's *File → Save Page As*.** That saves an HTML web page with a `.bin`
> name. It will not flash, and it fails in a way that looks like a broken board rather than a bad
> download — people lose hours to this.
>
> Or clone the whole repo and take the file from the folder:
> ```
> git clone https://github.com/monterman/BREmote-V2.git
> ```
>
> **Sanity check:** a real firmware image is a **few hundred KB**. If your file is a few KB, or opens
> as a web page in a text editor, you saved the HTML — go back and use the raw download.

---

## Step 2 — Know which board you're holding

**The TX and the RX use the same chip.** Nothing stops you flashing RX firmware onto the TX, and the
result is a board that boots and does nothing sensible. COM port numbers also move around between
reboots, so they are not a reliable way to tell the boards apart.

Plug in **one board at a time**, and confirm it by MAC address:

```
esptool --chip esp32c3 --port COM<N> read-mac
```

Write down which MAC is the TX and which is the RX. Do it once and you never have to wonder again.

If you would rather not install `esptool` at all, the safe habit is: **connect one board, flash it,
unplug it, then connect the other.** Never have both plugged in while flashing.

---

## Step 3 — Flash it

> 🎥 **[Watch Ludwig do exactly this, from 40:00 →](https://youtu.be/r6JIZEq3aTU?t=2400)** — same tool, same
> steps. Follow along with your own `.bin` and the settings below.

> **"Download" means upload.** The tool is writing *into* the board over the COM port. There is
> nothing being downloaded from the internet — ignore the name, it trips everyone up once.

1. **Unzip and run** `flash_download_tool_x.x.x.exe` (Ludwig's video shows **V3.9.8**).
2. In the first small window, choose:
   - **ChipType:** `ESP32-C3`
   - **WorkMode:** `Develop`
   - **LoadMode:** `UART`

   Press **OK**. The title bar should then read **ESP32C3 FLASH DOWNLOAD TOOL**.
3. Stay on the **SPIDownload** tab. Work along the **first file row, left to right** — the row reads
   `☐ [file path] … @ [address]`:

   | Order | Control | Set it to |
   |---|---|---|
   | 1 | **☐ checkbox** (far left) | **☑ ticked.** An unticked row is silently skipped — the tool runs, goes green, says FINISH, and writes nothing |
   | 2 | **file path** (via the `…` button) | your `.bin` |
   | 3 | **`@` address box** (far right) | **`0x10000`** ⚠️ blank by default — **this is the one people get wrong** |

   Leave every other file row empty and unticked.

4. Below that, in **SPIFlashConfig** — again in the order they sit on screen, left to right:

   | Order | Control | Set it to |
   |---|---|---|
   | 1 | **SPI SPEED** | `40MHz` |
   | 2 | **SPI MODE** | `DIO` |
   | 3 | **DoNotChgBin** | ☑ **ticked** |
   | 4 | **LockSettings** | unticked (greyed out) |

   Ignore **CombineBin** and **Default** — you do not need either.

5. Bottom of the window, in **DownloadPanel 1**, left to right:

   | Order | Control | Set it to |
   |---|---|---|
   | 1 | **COM** | the port your board is on |
   | 2 | **BAUD** | `115200` — what the video uses and the one that always works. `921600` writes faster; drop back the moment it misbehaves |

6. Press **START** (bottom-left, beside STOP and ERASE).

The **DetectedInfo** panel fills in as it connects — flash vendor, device ID, `QUAD;4MB`, crystal
`40 Mhz` — and the panel prints the board's MAC addresses. Seeing those means it is talking to the
chip properly.

Wait for the cyan **FINISH / 完成** box and a full green progress bar. Unplug and replug the board to
boot the new firmware.

> ### ⚠️ `0x10000` is the setting that matters
>
> These are **app-only** images. At `0x10000` the flash leaves the partition table and SPIFFS alone.
>
> **Never flash at `0x0`**, and ignore any reference to a `.merged.bin` — merged images are
> deliberately not published here. A full-chip write at `0x0` **erases your SPIFFS**: pairing,
> compass calibration and every setting, gone.

---

## Step 4 — Will this wipe my settings?

Sometimes, and it is worth knowing before you press START rather than after.

The flash itself does not touch your config. But **on boot the firmware compares the stored config
version against its own, and rewrites config to defaults when they differ.** So:

| You are flashing | Result |
|---|---|
| **The same `SW_VERSION`** your board already runs | Settings kept |
| **A different `SW_VERSION`** (newer *or* older) | **Config reset to defaults** — re-pair, re-calibrate, reconfigure |

Rolling back and then coming forward again wipes it **twice**.

Check what your board is running with **`?conf`** over serial before you flash, and
**[back up your config first](../README.md#-backing-up-your-settings--read-this-before-you-flash)**.

On the RX, a wipe costs you the compass calibration, which can only be restored by physically
re-running `?compasscal` and turning the buggy through two full circles. Back it up.

---

## When it doesn't work

| Symptom | Cause |
|---|---|
| Tool cannot find the port / no COM appears | **Charge-only USB cable**, or missing USB-serial driver |
| Connect fails, or fails partway | Use **BAUD `115200`**; try a different USB port; avoid hubs |
| Flashes fine, board does nothing | Check the **address was `0x10000`**, and that you flashed the right image for the board |
| "Success" / FINISH but nothing changed | The **file row checkbox was not ticked**, or the **`@` address box was blank** — both let it "succeed" having written nothing |
| Board behaves oddly after flashing | You may have cross-flashed TX firmware to the RX or vice versa — confirm by MAC and reflash |
| Downloaded file is only a few KB | You saved the GitHub **web page**, not the raw file — see Step 1 |

---

## After flashing

Go to **[Zero → Foiling](ZERO_TO_FOILING.md)** and continue from **§2.2 First boot & TX
calibration**. It walks you through calibration, pairing, compass, VESC/ESC setup, the wheels-up
safety check and your first session, in order.

---

## Related

- [Zero → Foiling — the full setup walkthrough →](ZERO_TO_FOILING.md)
- [Beta testing sheet →](Beta_Testing_Sheet.md)
- Advanced, only if you are changing the firmware:
  [Flashing the RX (Arduino)](FLASHING_RX_ARDUINO.md) · [Flashing the TX (Arduino)](FLASHING_TX_ARDUINO.md)
