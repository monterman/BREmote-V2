# Mounting the GPS + compass OUTSIDE the enclosure

**The single most effective fix for compass error on this buggy.** Everything else — calibration,
software filtering, tuning — works around the problem. This removes it.

> **Rule of thumb:** module **outside** the box, **no more than 10 inches** of cable, **fully
> waterproofed**, **nothing ferrous** anywhere near it.

---

## Why this matters more than anything else you can do

Measured on a real buggy with the module inside the enclosure, using `?magtest` under load:

| Motor current | Heading error |
|---|---|
| Idle | **3.2°** — rock steady |
| 5–15 A (ordinary cruise) | **87°** |
| 25–40 A | **101°** |

**The error does not scale with current.** It is already 87° at the first few amps and barely
grows after that. It *saturates*, because the interference field is already far stronger than the
earth's field — the needle stops measuring the earth and starts pointing at the motor.

That makes this a **geometry problem, not a current problem.** It cannot be ridden around, and it
**cannot be calibrated out** — calibration removes *fixed* errors, and this one moves with the
throttle. Distance is the only lever.

### The worst mistake: mounting over a loop in the phase wires

If your phase wires **U-turn** back over the ESC and the module sits above that turn, you are in
the single worst position possible.

Out-and-back conductors cancel each other's field **at a distance** — but at the **centre of the
loop they add**, and the bundle stops behaving like two wires and starts behaving like a magnet.
This is easy to create by accident and it explains error of the magnitude in the table above.

Getting out of that loop is worth roughly a *hundred-fold* reduction, because a loop's field falls
away with the **cube** of distance. **This is why 10 inches is enough and you do not need to mount
it on the nose.**

---

## Why 10 inches, and not longer

Two independent limits happen to land in the same place.

**1. I²C will not go far.** The GPS talks over a serial line and would happily run a metre. The
compass talks over I²C, which can only pull the line *down* and relies on a passive resistor to
pull it back *up*. A longer cable takes longer to charge, and past a point the receiver samples
the bit before it has arrived.

**Shielding makes this worse, not better** — a shield is a conductor right beside the signal
wires, so shielded cable has roughly double the capacitance per foot of loose wire.

| I²C bus speed | Practical maximum |
|---|---|
| **400 kHz** (this firmware's default) | **~12 in** — 10 in leaves useful margin |
| 100 kHz | ~3 ft |

If you must go longer than about 12 inches, drop the bus with `Wire.setClock(100000)` in
`Init.ino`. Nothing on that bus needs the speed.

**2. You do not need longer.** See the loop explanation above — 10 inches out of the loop already
gets you most of the available improvement. Extra cable buys little and costs reliability.

**Symptoms of an over-long I²C run** read like a dead module, not like a wiring problem: `?i2c`
finding the compass only sometimes, occasional garbage readings, calibration failing for no
visible reason.

---

## Waterproofing — the module is outside now

Once the module leaves the box it is exposed to spray, immersion and, if you ride salt, the most
aggressive environment there is. **Salt water does not evaporate away — it leaves conductive salt
behind and keeps corroding after it dries.**

### The GPS antenna must see the sky

**Never put the module in a metal box, and never wrap it in copper tape.** The GPS patch antenna
needs a clear view upward. Metal — including the shielding tape used on the *cable* — will cost
you satellite lock entirely.

**Plastic only:** ABS, polycarbonate, or clear epoxy. All are transparent to GPS.

### Three approaches, best first

| Method | How | Notes |
|---|---|---|
| **Pot it** | Set the module in clear epoxy or marine silicone inside a small plastic shell | **Most reliable.** Nothing to leak. Permanent — you cannot service the module afterwards. |
| **Small plastic enclosure** | ABS/polycarbonate box, cable in through a gland, lid sealed | Serviceable. The gland and the lid seam are the two things that will fail. |
| **Conformal coat + heatshrink** | Coat the PCB, then adhesive-lined heatshrink over the whole module | Lightest and cheapest. Adequate for spray, **not** for immersion. |

**Do all of them in layers if you ride salt.** Conformal coating on the bare PCB costs almost
nothing and is a good first line even under potting.

### The connector is where it will leak

The JST plug on the module is the weakest point in the whole assembly.

- **Best:** cut the connector off and solder the cable directly, then seal the joint.
- **If you keep it:** fill the shell with dielectric grease and cover the whole plug with
  adhesive-lined heatshrink.
- Seal **after** you have tested the wiring, not before.

### Cable entry

- Use a proper **cable gland** or a well-sealed grommet. A drilled hole and a blob of sealant will
  eventually wick water along the cable.
- **Exiting through the lid** is good — fewer bends, and it puts vertical distance between the
  cable and the phase wires, which run low. Just seal it properly, since a lid penetration catches
  spray from above.
- Leave a **drip loop** — a low point in the cable *before* it reaches the module or the box — so
  running water drips off instead of tracking into the seal.

---

## Nothing ferrous near the module

**You have just spent all this effort moving the compass away from one magnetic source. Do not
then bolt it down with another.**

- ❌ Zinc-plated steel screws, steel washers, steel brackets, hose clamps
- ❌ Anything with a magnet in it
- ✅ **Nylon or plastic** screws and standoffs
- ✅ **Brass** or genuinely **non-magnetic 316 stainless**

**Test everything with a fridge magnet before it goes on the buggy.** If the magnet sticks, do not
use it. This takes ten seconds and catches "316 stainless" that is nothing of the sort — a very
common problem with cheap fasteners.

The same applies to whatever the module is mounted *to*. A plastic bracket on a steel frame member
still puts steel under the compass.

---

## Mounting orientation

Mount the module **square to the nose** — its own forward axis lined up with the buggy's nose, or
turned exactly 90°, 180° or 270° from it. **Never diagonal.**

The firmware stores the mounting rotation only as one of those four values, so a module at, say,
30° gets stored as 0° and keeps 30° of heading error that **no calibration can remove.**

**Connector facing backwards** is a good convention: it is unambiguous, repeatable, and you can
confirm it at a glance without remembering which way the chip is oriented.

> A tester with a module turned about three quarters of the way round measured **251° of heading
> error** after a calibration that reported success. The old calibration could not detect
> rotation, because turning the module leaves the calibration circle perfectly round and centred.

---

## Cable routing and shielding

Shielding the cable protects the **data lines** from electrical noise. It does **nothing** for the
magnetic error — only distance fixes that. Both are needed, for different reasons.

**Full procedure — copper tape, twisting, and which end to ground:**
see **[BN-880 → RX wiring § Moving the module out on a cable](GPS_Wiring_BN880_RX.md)**.

The short version:

- Copper tape, **not** aluminium — copper tape has conductive adhesive so the seams conduct
- **Twist** the signal pairs before wrapping
- Ground the shield at the **RX end only** — grounding both ends creates a loop
- Insulate the outside so bare copper cannot short anything
- Route away from the phase wires and battery leads, especially in the first few inches

---

## After the move — this part is not optional

**Moving the module invalidates your stored calibration.** The hard-iron offsets are raw counts
tied to the old position next to those exact wires. The mounting angle has changed too.

**Until you recalibrate, the compass will be MORE wrong than before you moved it**, and the
firmware will notice: the compass and GPS course will disagree, the heading-disagreement latch
will set, **Follow-Me will refuse to engage** and RTM will drop to GPS-course-only. That is the
guard working correctly, but it looks exactly like a firmware fault if you are not expecting it.

### Do these in order

1. **Flash the current RX firmware** (do this first, so you calibrate on what you will ride).
2. **Move and mount the module** — square, connector back, non-magnetic hardware, sealed.
3. **`?compasscal`** — nose on **NORTH**, two full **CLOCKWISE** circles, finish on north.
   Watch the BIND LED:
   - **2 quick flashes** (*blip-blip*) — full success
   - **3 slow pulses** — PARTIAL: iron calibration saved, **mounting angle NOT updated.** Walk it
     again.
   - **10 fast flashes** — rejected, nothing saved
4. **`?magtest`** with the motor **under load** — prop in a bucket of water or held against the
   dock. It prints a verdict. ⚠️ **A free-spinning prop reads clean on a compass that is 100° out**
   — the command now refuses to grade a run under 5 A peak.
5. **`?compassheading`** pointing north, east, south, west. You want roughly 0 / 90 / 180 / 270.

### What good looks like

| Check | Target |
|---|---|
| `?magtest` verdict | **GOOD** (worst error under load < 10°) |
| N/E/S/W headings | within ~10° of 0 / 90 / 180 / 270 |
| Error pattern | if all four are off by *the same* amount, that is mounting rotation — re-run `?magalign` |

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `?i2c` finds the compass only sometimes | Cable too long for 400 kHz, or a poor joint. Try `Wire.setClock(100000)`. |
| Compass reads but headings are wrong in every direction by the same amount | Mounting rotation — run `?magalign` |
| Headings wrong by *different* amounts per direction | Iron calibration — run a full `?compasscal` |
| `?magtest` still reports large error after moving | Something ferrous nearby, or still too close to the loop. Check every fastener with a magnet. |
| Follow-Me will not engage after the move | You have not recalibrated. The disagreement latch is standing — `?diag` confirms it. |
| GPS lost satellites after the move | The module is shielded or under metal. **Never wrap the module itself.** |

---

## See also

- [BN-880 → RX wiring](GPS_Wiring_BN880_RX.md) — pinout, shielding procedure, cable length detail
- [Zero → Foiling](ZERO_TO_FOILING.md) — full setup walkthrough including compass calibration
- [GPS configuration](GPS.md) · [GPS troubleshooting](hardware/gps-troubleshooting.md)
