# TX GPS — Wiring & Troubleshooting Guide

**Who this is for:** First-time builders connecting a GPS module to the BREmote TX unit.  
**When to use it:** GPS dot on TX display blinks slowly and never goes solid, or you are wiring the GPS for the first time before potting.

---

## **What you need**

- BREmote TX unit powered on via USB  
- Arduino IDE with Serial Monitor open at **115200 baud**  
- GPS module (BN-220 recommended)  
- 4 small wires

---

## **Step 1 — Wire the GPS module**

You need **4 wires** between the ESP32-C3 board and the GPS module.

| Wire color | From ESP32-C3 | To BN-220 |
| :---- | :---- | :---- |
| 🔴 Red | **3.3V** | **VCC** |
| 🟡 Yellow | **UART G18** (Transmit Pin) | **GPS RX** |
| ⬜ White | **UART G19** (Receive Pin) | **GPS TX** |
| ⬛ Black | **GND** | **GND** |

### **All 4 wires run straight across — no crossing needed**

The BREmote PCB designer routed the UART crossover on the board itself. This means:

- GPIO19 (TX) connects to GPS TX — same label to same label  
- GPIO18 (RX) connects to GPS RX — same label to same label  
- All wires run in parallel

**Note for experienced builders:** This is the opposite of the usual UART convention where TX crosses to RX. The BREmote hardware handles this internally on the PCB traces — do not cross these wires.

---

## **Step 2 — Check the GPS LEDs before testing software**

Before opening the Serial Monitor, look at the GPS module LEDs:

| LED | What you see | What it means |
| :---- | :---- | :---- |
| Blue | Blinking | ✅ GPS is powered and sending data |
| Blue | Off | ❌ GPS has no power — check VCC and GND wires |
| Red | Off | GPS searching for satellites — normal indoors |
| Red | Blinking | ✅ GPS has a fix — satellites locked |

**If blue LED is off → fix power wiring first before proceeding.**

---

## **Step 3 — Open Serial Monitor and exit the charge screen**

1. Connect TX to your computer via USB  
2. In Arduino IDE: **Tools → Serial Monitor**  
3. Set baud rate to **115200** (bottom-right of Serial Monitor)  
4. You will see the TX boot messages

If the TX shows a charging screen, type this command and press Enter:

?exitchg

**Expected output after `?exitchg`:**

Starting Radio... Power: 20 Region: US/AU915 TOA: 7488 Done

Web config AP started. SSID: BREmoteV2-TX-WebConfig

IP: 192.168.4.1

TX GPS \[BN-220\]: connecting at 9600...

TX GPS \[BN-220\]: init complete (115200, 5Hz, NMEA filtered)

If you do **not** see the GPS init lines, type:

?gpsreinit

**Expected output after `?gpsreinit`:**

Re-running initTxGPS()...

TX GPS \[BN-220\]: connecting at 9600...

TX GPS \[BN-220\]: init complete (115200, 5Hz, NMEA filtered)

tx\_gps\_initialized after reinit: YES

Run ?gpsraw to check Serial1 output.

---

## **Step 4 — Check GPS status with `?printgps`**

Type this command and press Enter:

?printgps

**Expected output — everything working correctly:**

\----- TX GPS Status \-----

gps\_en:             1

gps\_chip\_type:      0

speed\_src:          5

tx\_gps\_initialized: YES

Serial1 available:  412

Chars processed:    44261

Sentences failed:   0

Location valid:     YES

Location age (ms):  14

Latitude:           41.936492

Longitude:          \-87.678212

Speed valid:        YES

Satellites:         12

HDOP:               75

tx\_gps\_speed:       0

\-------------------------

**What each field means:**

| Field | Good value | Problem value | What to do | |---|---|---| | `tx_gps_initialized` | `YES` | `NO` | Type `?gpsreinit` | | `Serial1 available` | \> 0 | `0` | GPS not sending — see Step 5 | | `Chars processed` | Increasing | Always `0` | GPS data not reaching ESP32 — see Step 5 | | `Sentences failed` | `0` | \> 0 | Baud rate wrong — check `gps_chip_type` in web config | | `Location valid` | `YES` | `NO` | Go outdoors and wait up to 2 minutes | | `Satellites` | 8 or more | `invalid` or \< 4 | Go outdoors — indoors gets poor signal |

---

## **Step 5 — Check raw GPS data with `?gpsraw`**

If `Serial1 available` was `0` or `Chars processed` was `0`, type:

?gpsraw 10

This listens on the GPS wire for 10 seconds and prints everything it receives.

---

**✅ Good — wiring correct, GPS working:**

\--- Raw GPS bytes from Serial1 (10 seconds, type q to quit) \---

$GNRMC,212045.80,A,4156.19047,N,08740.68690,W,0.231,,240426,,,A\*75

$GNGGA,212045.80,4156.19047,N,08740.68690,W,1,11,1.08,195.1,M,-33.8,M,,\*7F

$GNGSA,A,3,29,05,20,15,21,18,23,26,,,,,1.88,1.08,1.54\*13

$GNRMC,212046.00,A,4156.19047,N,08740.68686,W,0.266,,240426,,,A\*7B

\--- End raw GPS dump \---

You can read clear text starting with `$`. GPS is alive and talking. Continue to Step 6\.

---

**❌ Problem — wires swapped or broken:**

\--- Raw GPS bytes from Serial1 (10 seconds, type q to quit) \---

\--- End raw GPS dump \---

Completely empty — nothing received at all.

**Fix:** Double check all 4 wires match the table in Step 1\. GPIO19→GPS TX, GPIO18→GPS RX. Then run `?gpsreinit` and `?gpsraw 10` again.

---

**❌ Problem — baud rate mismatch:**

\--- Raw GPS bytes from Serial1 (10 seconds, type q to quit) \---

▒b\\▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ N▒▒▒{▒b\\▒▒▒▒▒▒▒▒▒▒▒

\--- End raw GPS dump \---

Garbage symbols — GPS is sending but at a different speed.

**Fix:** Go to web config at `http://192.168.4.1` (connect to WiFi `BREmoteV2-TX-WebConfig`). Set `gps_chip_type`:

- BN-220 → `0`  
- M10 or M100 Mini → `2`

Save, reboot, test again.

---

**❌ Problem — GPS not powered:**

Same as broken wire — empty output. Check: is the **blue LED blinking**? If not, the VCC or GND wire is missing or loose.

---

## **Step 6 — Get a GPS fix outdoors**

Once `?gpsraw` shows clean sentences, the wiring is correct. Now get a satellite fix:

1. Take the TX unit **outside** with a clear view of the sky  
2. Wait up to **2 minutes** for the first fix  
3. The **red LED** on the GPS will start blinking when satellites are locked  
4. Run `?printgps` — you should see `Location valid: YES` with real coordinates

**Verify:** Compare the coordinates in `?printgps` against a GPS app on your phone. They should match within about 5 meters.

---

## **Step 7 — Pre-potting checklist**

**Do not seal the unit until every item is ticked:**

- [ ] Blue LED on GPS is blinking  
- [ ] Red LED on GPS is blinking  
- [ ] `?printgps` shows `tx_gps_initialized: YES`  
- [ ] `?printgps` shows `Serial1 available` \> 0  
- [ ] `?gpsraw 10` shows readable `$GNRMC` sentences  
- [ ] `?printgps` shows `Location valid: YES`  
- [ ] Latitude and longitude match your phone within 5 meters  
- [ ] `Sentences failed: 0`

---

## **Common failures at a glance**

| Symptom | Most likely cause | Fix |
| :---- | :---- | :---- |
| GPS dot blinks slowly forever | No satellite fix | Go outdoors, wait 2 minutes |
| `Serial1 available: 0` | Wrong wiring or broken wire | Check Step 1 wiring table |
| `Sentences failed` \> 0 | Wrong `gps_chip_type` | Set `0` for BN-220 in web config |
| `tx_gps_initialized: NO` | Charge screen blocking boot | Type `?exitchg` first |
| Blue LED off | No power to GPS | Check VCC and GND wires |
| Garbled `?gpsraw` output | Baud rate mismatch | Change `gps_chip_type` in web config |
| Worked before potting, dead after | Wire broken inside compound | Open unit, retest with `?gpsraw 10` |

---

## **Potting tips**

1. **Test fully before potting** — once sealed you cannot see the GPS LED  
2. **Leave a small loop of wire** at each solder joint — prevents tension pulling joints loose during pour  
3. **Use low-viscosity compound** around the GPS module — thick epoxy can stress PCB as it shrinks  
4. **Do not pot in cold temperatures** — compound cures slowly below 15°C  
5. **Test again immediately after curing** — before final assembly

---

## **Supported GPS modules**

| Module | Voltage | Works on TX? | Notes | |---|---|---| | **BN-220** | 3.3V | ✅ Confirmed | Recommended. Set `gps_chip_type=0` | | **M100 Mini** | 3.3–5V | ✅ Expected | For Nano TX. Set `gps_chip_type=2` | | M100 Pro | 3.6–5.5V | ❌ No | Needs more than 3.3V — UART corrupted at 3.3V | | M101Q / M10Q | 5V | ❌ No | Needs 5V — TX only provides 3.3V |

---

## **Serial commands quick reference**

| Command | What it does |
| :---- | :---- |
| `?exitchg` | Exit the charge screen so boot completes |
| `?printgps` | Full GPS status snapshot |
| `?gpsraw 10` | Dump raw GPS bytes for 10 seconds |
| `?gpsreinit` | Re-run GPS initialization without rebooting |

---

*Guide created: 2026-04-24 — based on real diagnosis during TX unit 1 build.*  
