# M100 Mini → TX Wiring (GPS, no compass) — remote shield and Nano

**Applies to:** V2.5-Evo TX with an **HGLRC M100 Mini** GPS (u-blox M10 class, no compass, 3.3–5 V,
4-pin SH1.0) — either the **V2.4 TopShield/BotShield remote** or the **Nano V2-2** board.
**Firmware pins on both:** `P_U1_RX 18` / `P_U1_TX 19` (`BREmote_V2_Tx.h`). Set `gps_chip_type` to **2 (M10)**.
The Mini talks 115200 natively, so the boot skips the 9600→115200 dance the BN-220 needed.

> Direction, once and for all: the **module's TX** goes to the ESP's **receive** pin **GPIO18**; the
> **module's RX** goes to the ESP's **transmit** pin **GPIO19**. That is how every working remote is wired.
> If you are replacing a BN-220 that works, go **wire for wire**: Mini white where the BN-220's TX wire was,
> Mini yellow where its RX wire was.

---

## A. Remote (V2.4 BotShield) — the 8-pad row beside the flex connector

Silkscreen labels as printed, reading from the pin-1 end: **`3V3 · G18 · G19 · RX · TX · SCL · SDA · GND`**.

| M100 Mini wire | BotShield pad (silkscreen) | Notes |
|---|---|---|
| **red** (V) | **`3V3`** | pad 1 of the row; the Mini accepts 3.3–5 V |
| **white** (M100 **TX**) | **`G18`** | GPIO18 = `P_U1_RX` — module talks, ESP listens |
| **yellow** (M100 **RX**) | **`G19`** | GPIO19 = `P_U1_TX` — ESP config writes |
| **black** (GND) | **`GND`** | pad 8, the far end of the row |

**Do not use the pads labelled `RX` / `TX`** in the same row — those are the CH340 USB-serial lines (UART0).
`SCL`/`SDA` beside them are the display/ADC I²C bus; the Mini has no compass, nothing goes there.
(`R23`/`R24` next to the row are 10 k pull-downs on G18/G19 — leave them.)

---

## B. Nano V2-2 — straight onto the module castellations

> The Nano has **no solder pads for GPIO18/19** — those two nets end at the module. The wires go
> straight onto the **HT-CT62 castellations**. The module is soldered on the **underside** of the
> Nano, so flip the board: the row you want is the one beside the `LORA` silkscreen.

### Pin map — the four wires (labels as printed on the HT-CT62)

Wire **label to label**. The pad numbers and GPIOs are for cross-checking against the schematic only.

| M100 Mini wire | HT-CT62 castellation (silkscreen) | Schematic pad · GPIO | Notes |
|---|---|---|---|
| **red** (V) | **`VDD`** | pad 12 | 3.3 V rail — the Mini accepts 3.3–5 V |
| **black** (GND) | **`GND`** — the one right beside `VDD` | pad 13 | any `GND` castellation works; this one is closest |
| **white** (M100 **TX**) | **`18`** | pad 17 · GPIO18 = `P_U1_RX` | module talks → ESP listens |
| **yellow** (M100 **RX**) | **`19`** | pad 18 · GPIO19 = `P_U1_TX` | ESP config writes → module |

**Do not use `TXD` / `RXD`** — they sit one pad past `19` and are the CH340 USB-serial lines (UART0).
Soldering the GPS there kills USB flashing and gives the firmware nothing.

Reading the LoRa-side column on the module underside, top to bottom (LoRa antenna corner first):
`LoRa ANT · GND · TXD · RXD · 19 · 18 · 8 · 9 · 10 · GND · VDD`. So from the `VDD` end: `VDD`, `GND`,
`10`, `9`, `8`, **`18`**, **`19`**, `RXD`, `TXD`, `GND`, `LoRa ANT`.

Power may equally be taken from the display's **`VCC` / `GND`** pads on the top side (same 3.3 V rail).

Render: `Electronics/TX_Nano_bottom_m100-uart-pads.png` (bottom view, board flipped, wires colour-marked).
