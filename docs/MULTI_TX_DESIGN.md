# Multi-remote design v3.1 ("hybrid")

> **Design v3.1 — NOT implemented, NOT tested. For review.**
>
> Nothing in this document exists in the firmware yet. No board has been flashed with it. It is
> published so the protocol, the safety rules and the test plan can be reviewed **before** any code
> is written. Comments are welcome in the forum thread "BREmote - Future Development".

**Base:** V2.5-Evo at commit `652112a`: receiver SW36, `confStruct` 200 bytes; remote SW27, 136 bytes. Line references below are at that commit.

**Proposed versions:** receiver SW37 (232 bytes) and remote SW29 (144 bytes). Both carry the old settings over (section 13).

---

## Credits

The ideas come from the forum discussion:

- **Jan12:**
  - each remote sends **its own address** in the three address bytes, so the receiver can tell remotes apart and accept or decline each one by its paired list;
  - the **"owning"** model: one remote in control at a time;
  - **E9** only on the remote that is being ignored, with the motors kept running;
  - a **"not in control" bit** in the telemetry index;
  - **slower idle remotes**;
  - a takeover that needs **zero throttle first, then a low limit**.
- **ludwig_bre:**
  - the receiver **answering each remote** individually;
  - the **collision backoff and hop** from V2.2.7, which every remote keeps using here.
- **Earlier multi-remote notes in this fork:**
  - master and guest roles (the first remote paired is the master);
  - one pairing window for all remotes;
  - minimum RF power while pairing.

A design review before code produced the v3.1 changes listed in section 15.

---

## 1. Scope

- **One buggy and up to two remotes.** The data structures keep room for four remotes, so a third remote later needs no layout change.
- **Several buggies near each other are not a supported use yet.** The packet format already keeps buggies apart (section 3), and a host test checks it. A separate design pass will cover that case.
- **With a single remote paired, nothing changes.** The receiver runs today's code path, byte for byte.

---

## 2. Summary

**Packet and identity:**
- **Packets stay 6 bytes.** No change in airtime for the rider in control.
- **Each board's identity comes from its chip** (the eFuse MAC), computed at every boot. It is no longer the stored `own_address`, so a copied configuration cannot clone a remote.
- **Bytes 0-2 of every new-style uplink = remote ID XOR buggy ID** (the "link ID"). The receiver recovers the remote's ID exactly and checks it against its paired list. A packet can drive only the buggy it was paired with, and a reply can never be mistaken for a control packet.

**Replies:**
- **The receiver answers every accepted remote.**
- The reply's index byte carries a 3-bit status: you control / free / busy / you may take it back / refused (E9) / taken from you.

**Ownership:**
- **One owner at a time.** Only the owner's packets can refresh the motor failsafe or set the throttle.
- With no owner, the motor gate is shut, exactly like today's failsafe.

**Takeover and release:**
- **Takeover only from zero throttle**, then a limit: 25 % at the moment of takeover, rising evenly to full over 4 s.
- **Release happens in one of three ways:**
  - by lock;
  - by 5 s at zero while another unlocked remote is on;
  - by losing the owner's signal for failsafe time + 5 s while another unlocked remote is on.
- **A spare remote that nobody touches for 120 s locks itself.** A locked remote cannot take control and does not count as "another remote on".
- **The master can take control back from a guest:**
  - throttle at zero;
  - toggle RIGHT tap, then RIGHT hold 1.2 s;
  - refused while Follow-Me or Return-To-Me is armed.
  - About 1.6 s to motor zero. The guest's remote shows "tA".

**Pairing:**
- One receiver window for all remotes. The list is kept in RAM and saved only when the window closes, so an aborted pairing never unpairs the buggy.
- Both ends pair at -9 dBm and accept only strong frames (≥ -60 dBm, about 5 m or less).

---

## 3. Identities and the link ID

| Name | Value |
|---|---|
| chip ID | `{crc8(mac[0..3]), mac[4], mac[5]}`, the same formula as today's default address, computed at every boot |
| buggy ID `R` | the receiver's chip ID. The stored `own_address` is forced equal to it. |
| remote ID `T` | the remote's chip ID |
| link ID `L` | `T XOR R`, byte by byte |

**Example with made-up IDs:**
- buggy `5C:10:20`, remote A `5C:4A:7B`, remote B `5C:91:0E`;
- `L` for A = `00:5A:5B`; `L` for B = `00:81:2E`.

The receiver computes `L XOR R` and finds the remote. Any other buggy computes `L XOR R'`, gets an ID that is not on its list, and drops the frame. A reply is addressed to `T`, which differs from `L` by `R`, never zero, so no buggy can read a reply as a control packet.

**Why not the plain remote address in bytes 0-2?** Then an uplink `[T][throttle][steering][CRC]` and a reply `[T][index][value][CRC]` have the same form:
- a buggy that still has remote A on its list would obey A after A was paired to another buggy;
- it would also obey the other buggy's **replies** to A as throttle commands. With the status bits in the index byte, that is up to about 83 % throttle with nobody touching a trigger.

XOR keeps Jan12's idea (one ID per remote, carried in every packet) and closes both cases, at no airtime cost.

**Identity is not confidential.** It is in every packet. The design protects against confusion (two buggies, two remotes, copied configurations), not against a deliberate attacker.

**If a buggy's stored address was ever edited by hand or copied from another board,** the first boot of the new firmware forces it back to the chip ID and **drops the old pairing**. A binding made for another identity is never kept. Pair once again.

---

## 4. Packet formats

### 4.1 Byte 3 of an uplink: the allocation table

This is the table referred to by `DESIGN_RETURN_TO_ME.md`.

**Today (implemented at `652112a`):**

| Byte 3 | Byte 4 | Meaning |
|---|---|---|
| `0x00-0xF0` | steering 0-254 (127 = centre) | throttle; the remote clamps to `0xF0` |
| `0xF1` | 0 off, 1 on, `0x02` refresh, `0x80-0xFF` remote boot ID | Return-To-Me state. **Every other value is ignored** by every receiver since 2026-04-25 (`Radio.ino:405-468`). |
| `0xF2` | mode bits 0-2, auto-return override bits 5-6, bit 7 fresh | Follow-Me declaration |
| `0xF3` | `0x03` (15-byte packet follows) or `0x01` (14-byte) | rider GPS announce |
| `0xF4` | flags | aux (light / find-me) |
| `0xF5-0xFF` | - | unallocated, never sent. A receiver at this commit reads them as throttle, so they must stay unused. |

**Proposed by v3.1 (not implemented):**

| Byte 3 | Byte 4 | Meaning | Sent by |
|---|---|---|---|
| `0xF1` | **`0x10`** | **LOCKED**: this remote is locked | new-style remotes only, always with a link-ID address |
| `0xF1` | **`0x5A`** | **SEIZE**: the master takes control back | the master only, same rule |
| `0xF5-0xFF` | - | still never sent; a v3.1 receiver **drops** them from every remote | - |

**Why LOCKED and SEIZE live inside 0xF1.** Any new code must be harmless on a buggy that never got the update.

| Receiver build | What `0xF1/0x10` or `0xF1/0x5A` does there |
|---|---|
| before 2026-04-25, and upstream receivers | throttle 241. This is the same as every existing 0xF1-0xF4 meta packet on those builds; only the address protects them, and a link-ID frame never matches it. |
| 2026-04-25 to 2026-09-28 | the value is ignored, but on these builds any meta packet refreshes the motor failsafe |
| 2026-09-28 (`f79a50f`) to today | ignored; a liveness stamp that cannot keep the motor running |

A new code in `0xF5`/`0xF6` would instead be 96 % throttle on every one of these builds.

**One frame builder.** On the remote, a single function produces address and byte 3 together. It is the only place that can emit LOCKED or SEIZE, and only with a link-ID address.

### 4.2 Uplink layout

| Byte | New-style link | Legacy link (today's frames) |
|---|---|---|
| 0-2 | `L = T XOR R` | the buggy's address |
| 3 | table above | throttle or `0xF1-0xF4`; **a locked remote sends 0 / 127**, as today |
| 4 | steering or the type's value | same |
| 5 | CRC8 over bytes 0-4 | same |

The rider-GPS packets keep their layout (announce 6 B, data 15 B), with `L` as the address when the link is new-style.

### 4.3 Reply, buggy → remote

| Byte | Content |
|---|---|
| 0-2 | the remote's ID |
| 3 | **bits 7-5 = status**, bits 4-0 = telemetry index 0-19 |
| 4 | the telemetry value at that index |
| 5 | CRC8 over bytes 0-4 |

| Status | Name | Meaning for the remote |
|---|---|---|
| 000 | OWNER_SOLO | you control, and you are the only remote. **The reply is then byte-identical to today's.** |
| 001 | OWNER | you control; other remotes are paired |
| 010 | FREE | nobody controls; take it from zero throttle |
| 011 | BUSY | another remote controls |
| 100 | SEIZABLE | a guest controls; you are the master and may take it back |
| 101 | REFUSED | another remote controls and your input is ignored → **E9** |
| 110 | SEIZED | the master took control from you → **"tA"** |
| 111 | reserved | read as BUSY |

**Each remote gets its own telemetry rotation.** With one shared rotation, a remote answered every 10th time would never see some indices.

**A remote that is not in control** gets the owner's Follow-Me and Return-To-Me fields blanked, so a guest's screen never shows someone else's mode. Battery, temperature, speed, water alarm and its own link quality are sent normally.

### 4.4 Pairing frames (byte 0)

| Frame | Today | v3.1 |
|---|---|---|
| request (remote → buggy, 5 B) | `0xAB` + address | `0xAD` + remote ID |
| answer (buggy → remote, 8 B) | `0xBA` | `0xDA` + remote ID + buggy ID |
| confirm (remote → buggy, 8 B, 3 times) | `0xAC` | `0xAE` + buggy ID + remote ID |

- A receiver that was not updated ignores `0xAD`, so a new remote cannot pair with it (the remote shows "EP"). To go back, restore the remote's saved configuration (`?conf` backup), which brings back its old link.
- A new receiver still accepts `0xAB` from an old remote, but **only as its single remote** (section 7).

---

## 5. The buggy (receiver)

### 5.1 Modes

| Mode | When | Behaviour |
|---|---|---|
| SINGLE | exactly one remote paired (old or new) | today's code path. A LOCKED frame is handled exactly like today's locked packet (0 / 127). Status always 000. No takeover limit. |
| MULTI | 2-4 remotes paired, **all new-style** | the ownership rules below |
| unpaired | no valid entry | nothing is accepted |

### 5.2 Per-packet rules (MULTI)

Each frame passes these checks in order:
1. the paired check;
2. a frame addressed `00:00:00` is dropped;
3. the remote is looked up by `L XOR R`;
4. `0xF5-0xFF` is dropped.

Then:

**Control packet from the owner:**
- applied exactly as today;
- it is the only path that writes the throttle and stamps the motor failsafe.

**Control packet while nobody owns:** control is granted when all of these hold:
- the remote's last two or more packets were at zero throttle, within 3 s;
- this packet is a squeeze (≥ 12 of 255);
- Follow-Me and Return-To-Me are not armed;
- the motor output is running.

The zero-proof counter is updated **after** that check. A held trigger never takes control.

**Control packet from another remote:**
- with input → REFUSED (E9);
- without input → BUSY, SEIZABLE or SEIZED.
- The owner's motors are not affected.

**LOCKED from the owner:** release at once. A locked frame never counts as zero proof, so an unlock squeeze cannot become a takeover.

**SEIZE from the master:** accepted when all of these hold:
- the owner is a guest;
- take-back is enabled;
- Follow-Me and Return-To-Me are not armed;
- the master sent zero throttle within the last second.

**Meta packets** (RTM, Follow-Me, GPS, aux): from the owner only. A guest's GPS announce never switches the receiver into 15-byte mode.

### 5.3 Release and takeover

**Release does this, in this order:**
1. throttle 0 and steering centre (no failsafe stamp, so the gate shuts on its own clock);
2. "RTM off" through the existing handler (which keeps the slow ceiling of a return that was running);
3. "Follow-Me off";
4. the rider-GPS state, the GPS cross-checks and the remote boot ID reset;
5. an owner-change counter.

The receiver's Follow-Me code watches that counter. It closes the short "how the last run ended" windows, so a new rider never sees the previous rider's stop on his screen.

**Takeover:**
- The **takeover limit** (section 6) is published first, then the new owner's throttle.
- The limit's start time is never stored as 0, because 0 means "no limit".

### 5.4 Release timers

- **"Another remote on"** = a remote other than the owner sent a control packet (not LOCKED) within the last 3 s.
- **Link loss:** the owner is silent for failsafe time + 5 s (6 s by default) while another remote is on.
- **Idle:** the owner has sent zero throttle and no steering for 5 s while another remote is on, and Follow-Me / Return-To-Me are not armed.
- **Without another remote on, the owner keeps control**, exactly as today. A solo rider is never released by a pause or a dropout.
- The timers may run late, never early. **Motor safety never depends on them:** the existing motor failsafe shuts at failsafe time on its own.

---

## 6. Motor path: the takeover limit

- One `min()` in the receiver's motor calculation, after the existing caps and before the output ramp: `throttle = min(throttle, limit)`.
- The limit is 64 of 255 (25 %) at takeover, rising evenly to full over 4 s: 64 / 111 / 159 / 207 / 255 at 0 / 1 / 2 / 3 / 4 s.
- Both numbers are settings. The time can never exceed 10 s.
- It only lowers throttle, never reads GPS, and is never used with a single remote.
- The motor failsafe expression, all existing caps, the ramp, the mixer and the steering are untouched.

---

## 7. Pairing

**Opening the window:** hold BIND at boot (unchanged).
- The window is 60 s, plus 20 s for each remote that pairs, 120 s at most.
- A BIND press closes it.

**Requests:**
- A new-style request is answered and confirmed (both frames name both IDs).
- An old-style request is accepted only if it is the first one, and it then closes the window: an old remote may only be the single remote.

**Refused:**
- an ID of `00:00:00`;
- the buggy's own ID;
- a full list;
- **a frame weaker than -60 dBm, checked on both ends.**

**At close:**
- With at least one remote paired, the new list (in pairing order; first = master) is saved once. The LED flashes N times per remote (N = its slot).
- With none, the old pairing is restored and nothing is written.
- The list is always rebuilt, never appended.

**Power and the RSSI gate:**
- Both ends pair at -9 dBm and restore their normal RF power on every way out.
- The remote's restore never wakes a radio that has just been put to sleep.

**Why -60 dBm.** At -9 dBm, free-space loss at 915 MHz (`20·log10(d) + 31.7 dB`) plus about 5 dB of hand and antenna loss gives:

| Distance | 1 m | 2 m | 5 m | 10 m | 15 m |
|---|---|---|---|---|---|
| expected RSSI | ≈ -46 dBm | ≈ -52 | ≈ -60 | ≈ -66 | ≈ -69 |

So -60 accepts about 5 m or closer, leaves 14 dB of margin at arm's length, and refuses 15 m. Reduced power alone is not proof of proximity: at -9 dBm a pairing can still complete at well over 100 m.

**Pair the remote you ride with first,** so it becomes the master.

---

## 8. The remote

### 8.1 Rate

- **The owner** sends every 100 ms, as today, with the collision backoff unchanged.
- **A remote that is not in control** drops to one packet per second when idle.
  - It stays slow even with input while another remote is in control. E9 still shows within about a second.
  - It goes fast when control is free and it is squeezed, at the first press of the take-back gesture, or when it has had no reply for 5 s.
- In slow mode a missed reply shifts the next send by 33 or 66 ms, never 0, so it cannot sit in the owner's slot.
- While slow, the "connected" window is 2.5 s instead of 1 s, and the urgent link-lost buzz is suppressed (the blink still shows).

### 8.2 Owner-only actions

- Follow-Me arm and mode changes, Return-To-Me arm, aux, and the rider-GPS stream need ownership.
- Without it the remote shows "St" and the stop buzz.

### 8.3 Lock and auto-lock

- A new-style remote sends LOCKED while it is locked.
- **Auto-lock for a remote that is not in control** (in multi-remote mode): 120 s by default (setting, 0 = off).
- **Auto-lock for a single remote, or the owner:** off by default (setting).
- Auto-lock is skipped while Follow-Me or Return-To-Me is armed.

### 8.4 Master take-back

**Offered when** the last two replies said SEIZABLE and the newer one is under 5 s old.

**Gesture:**
1. Trigger released.
2. RIGHT tap.
3. RIGHT pressed again within 400 ms and held for 1.2 s (setting, 1.0-1.8 s).

**While the gesture is armed:**
- the remote is fast;
- the normal 2 s RIGHT-hold display cycle is suppressed for that press.

**Aborts:** any throttle, an early release, or a status change.

**On completion** the remote sends SEIZE three times. The receiver re-checks every condition itself.

**About 1.6 s from the first press to motor zero.** An accidental take-back can only slow the guest's ride to a stop; it can never start the buggy.

### 8.5 Screens

- **E9**: refused. One buzz on the rising edge. This is not the remote's error state, so the next toggle press is not taken as an error acknowledgement.
- **"tA"**: control was taken. Shown 3 s, with one buzz.
- **Ownership dot:** shares the Follow-Me status pixel and shows only while the Follow-Me dots are off. OFF solo, SOLID owner, PULSE free, SLOW busy.

### 8.6 Smaller fixes on the remote

- The telemetry wait keeps listening after a frame meant for someone else, until its own reply or 40 ms after the send. It never touches the radio after that bound.
- The random seed of the collision hop comes from the chip ID, so two remotes with copied configurations do not hop in step.

---

## 9. Old and new mixed

| Buggy | Remote | Result |
|---|---|---|
| old | old | today |
| old | new, with its migrated legacy link | works as today (today's frames; locked = 0 / 127) |
| old | new, tries to pair | no pairing ("EP"); restore the remote's backup to go back |
| new (migrated) | old, existing pairing | works as today, **if** the buggy's stored address equals its chip ID; otherwise pair again |
| new | old, paired in a new window | accepted as the single remote |
| new | new | full v3.1 |
| new | two old remotes that were cloned to one address before the update | still look like one remote (today's behaviour). Update both and pair them in one window. |
| any buggy | a frame for another buggy | ignored (the link ID does not decode to a paired remote) |
| new | a remote moved to another buggy | ignored by its old buggy, though still on its list |
| new | a remote with a copied configuration | identifies as itself (chip ID) |
| new, with a configuration copied from another board | - | its pairing is dropped at boot |
| downgraded | - | version mismatch → defaults → unpaired |

---

## 10. Failsafe paths

| Case | What stops the motors | When another remote may take control |
|---|---|---|
| Owner goes silent (switched off, out of range) | the motor failsafe at failsafe time after its last control packet; other remotes' packets cannot refresh it | after failsafe time + 5 s, only if another unlocked remote is on; from zero, under the limit |
| Owner comes back within that time | resumes from 0 with the soft ramp | - |
| Owner locks | throttle 0 at the first LOCKED frame (after the lock flush, about 300 ms) | at that frame, from zero, under the limit |
| Owner idles 5 s at zero, another remote on, no FM/RTM | already at zero | at once, from zero, under the limit |
| Receiver reboots with several remotes paired | no owner, gate shut | the first remote with zero proof and a squeeze; **a held trigger never restarts the buggy** |
| A guest squeezes while the master rides | nothing; the guest sees E9 | never from an active master |
| Master take-back | output to 0 at once | master at zero + gesture + FM/RTM off |
| Control changes while FM/RTM was armed | FM/RTM end on the buggy; rider GPS and checks reset; the old rider's stop notices are cleared | after Follow-Me is idle (≤ 100 ms) |
| **Unattended spare remote** | locks itself after 120 s; a locked remote is not "on", so the owner is never released and the spare cannot take control | in the first 120 s after it was last touched: an owner pause can release, and a squeeze on the spare takes control under the limit. The master's take-back stops it in about 1.6 s. **If the rider's own remote is the guest, it cannot stop a spare that stays squeezed**, which is why the rider's remote is paired first. |
| A stranger's remote during the pairing window | the RSSI gate (≥ -60 dBm, about 5 m) on both ends; LED count and `?mtx` list after the window | a stranger within about 5 m during the window remains possible; pair one buggy at a time |
| Unpaired buggy | nothing accepted | - |
| Unpair paths (binding reset on import, `paired = 0`, factory reset) | the remote list is cleared with them | - |
| A copied or edited buggy configuration | identity from the chip; the copied pairing is dropped | - |
| Corrupt reply | the buggy decides; a remote needs 3 non-owner replies in a row before it slows down or disarms | - |
| Accepted, unchanged from earlier notes | if the master's remote fails **while still transmitting** (an undetected stuck input), a guest cannot take over | only once it goes silent or the input fault fires |

---

## 11. Interactions

**Rider GPS:**
- Only the owner's stream is sent and accepted.
- On a change of owner the GPS state and cross-checks reset.
- The existing GPS integrity rules hold: every fix is judged, the 0xF3 sequence resets on any mismatch, int32 microdegrees are kept, and nothing is extrapolated.

**Follow-Me declarations** are per owner. A new owner arms with its own gesture.

**Front stations (F4 / F5):** every remote paired to one buggy must run the same remote build.

**Single-core chip:** no new tasks and no core pinning. All ownership state has one writer (the radio task).

---

## 12. Airtime (EU 869.4-869.65 MHz, 10 % per transmitter)

6-byte packet = 7.488 ms, 15-byte = 12.864 ms (SF6, BW 250 kHz, CR 4/7). An idle remote = one packet per second.

| Remotes on | Rider GPS stream | Owner remote | Each idle remote | Buggy |
|---|---|---|---|---|
| 1 | on | 10.06 % | - | 5.99 % |
| 2 | on | 10.06 % | 0.75 % | 6.74 % |
| 2 | off | 7.49 % | 0.75 % | 8.24 % |
| 3 | off | 7.49 % | 0.75 % | 8.99 % |

**Notes:**
- The owner remote is already at about 10 % with the 2 Hz GPS stream today, independent of this design. EU users: GPS stream off or a lower GPS rate.
- Each reply makes the receiver deaf for about 21 ms. Three idle remotes make it deaf about 6 % of the time, far inside a failsafe that needs ten owner packets lost in a row.
- **US 902-928 MHz:** no duty limit. A fixed 250 kHz channel does not fit FCC 15.247 as it stands (lower power or duty, or FHSS, or a 500 kHz preset). This design neither fixes nor worsens that.

---

## 13. Configuration

**Receiver, SW36 → SW37, 200 → 232 bytes (appended):**

| Field | Default | Range | Purpose |
|---|---|---|---|
| `mtx_id[12]`, `mtx_flags[4]` | 0 | written by pairing only | up to 4 remote IDs in pairing order (slot 1 = master); per slot: valid, legacy |
| `mtx_release_s` | 5 | 3-60 s | idle time before release, and the extra time on a link loss |
| `mtx_seize_en` | 1 | 0-1 | master take-back allowed |
| `mtx_takeover_cap` | 64 | 0-255 (255 = off) | throttle limit at takeover |
| `mtx_takeover_ms` | 4000 | 0-10000 ms | time for the limit to reach full |
| `mtx_rsvd[4]` | 0 | - | reserved |

**Remote, SW27 → SW29, 136 → 144 bytes (appended; SW28 is never used):**

| Field | Default | Range | Purpose |
|---|---|---|---|
| `autolock_timeout_s` | 0 | 0, 5-180 s | auto-lock for a single remote or the owner |
| `mtx_link_proto` | 0 | 0 or 3, written by pairing | legacy or new-style frames |
| `mtx_seize_hold_ms` | 1200 | 1000-1800 | hold time of the take-back |
| `mtx_autolock_s` | 120 | 0, 30-600 s | auto-lock while not in control, in multi-remote mode |

**Rules:**
- **Both boards migrate their old configuration** by the existing one-step prefix migration, extended to the remote. Settings, throttle calibration and compass calibration are kept.
- The pairing is kept only where the stored address equals the chip ID.
- The cross-field validator runs on every intake path:
  - `paired = 0` clears the remote list;
  - a stored address that is not the chip's own drops the binding;
  - a legacy entry next to a new-style entry is removed;
  - a link mode other than 0 or 3 unpairs the remote.
- `own_address` becomes read-only on the receiver.
- New commands: `?mtx` on both boards (the list, roles, owner, status, rates, counters) and `?mtxdot` on the remote (dot test).
- Every new field goes into both web pages and the web serial config tool.
- `TelemetryPacket`, the logging struct, the packet size and the radio settings do not change.

---

## 14. Tests before any flash, then the bench

### 14.1 Host tests (`Tools/tests/multi_tx_test.cpp`, against a pure `Source/Common/MultiTx.h`)

| # | Covers |
|---|---|
| T1 | link ID round trip; never zero; uplink ≠ reply address |
| T2 | classification on two buggy IDs; a frame for one buggy is unknown on the other (this is the only coverage of the multi-buggy case for now) |
| T3 | byte-3 decode, including LOCKED / SEIZE exactly, 0xF5-0xFF dropped |
| T4 | takeover: zero, zero, squeeze → yes; held trigger → never; FM armed → no; FM never declared → allowed |
| T5 | take-back conditions |
| T6 | status priority for a remote that is not in control |
| T7 | release timers, with and without another remote; across the `millis()` wrap |
| T8 | the takeover limit curve and its edge cases |
| T9 | status packing; status always 000 with one remote |
| T10 | the telemetry masks |
| T11 | per-remote telemetry rotation |
| T12 | entry and binding validation |
| T13 | the pairing window, including the RSSI gate |
| T14 | old/new mixes |
| T15 | the remote's single frame builder (no LOCKED / SEIZE outside a link-ID address, nothing ≥ 0xF5) |
| T16 | remote status, rate, auto-lock and take-back logic |
| T17 | link-fresh windows |
| T18 | the bounded telemetry wait |
| T19 | configuration migration on real configuration exports of each board. These are kept locally because they contain pairing data, and are not committed. |
| T20 | the paired check and every unpair path |
| T21 | the release order and the clearing of the previous rider's stop notices |

### 14.2 Builds

- **Receiver:** `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (it uses its own `partitions.csv`; never `huge_app`).
- **Remote:** `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app`.
- **The classic remote and the Nano remote build from the same sketch, the same FQBN and the same binary.** Their differences (GPS chip, calibration, magnet sensor) are configuration.
- Zero errors, zero new warnings.

### 14.3 Bench (props off, motors secured, every other receiver switched off)

**Phase A: receiver updated, remotes still old.**
- migration and identity notes;
- old-style pairing as the single remote;
- the single-remote regression (identical to today);
- an aborted window keeps the pairing and restores the RF power.

**Phase B: first remote updated.**
- remote migration, with no calibration prompt;
- a locked legacy-link remote still sends 0 / 127;
- new-style pairing: refused at 15 m, accepted at 1 m, RSSI recorded on both ends.

**Phase C: both remotes updated, paired in one window (rider's remote first).**
- ownership and dots;
- E9;
- release by idle (with the presence rule), by lock and by auto-lock;
- takeover from zero under the limit;
- a guest can never take from an active master;
- the master take-back (≤ 2 s);
- the owner switched off mid-throttle: the failsafe shuts in ≤ 1.1 s, and nothing from the other remote reopens it;
- the idle rate over 10 min, with no urgent buzz;
- Follow-Me / Return-To-Me ownership;
- airtime;
- the unattended spare;
- an owner change right after a Follow-Me fault stop;
- the unpair paths.

**Then the same core cases on the main buggy**, and the bench receiver's list is cleared.

---

## 15. What changed from v3

- The XOR link ID is mandatory.
- LOCKED and SEIZE moved from `0xF5` / `0xF6` into `0xF1` (section 4.1). There is one frame builder, the link mode is clamped to {0, 3}, and 0xF5-0xFF is never sent and always dropped.
- Identity comes from the chip before the configuration loads. A binding made for another identity is dropped.
- "Follow-Me armed" means modes 1-5; "never declared" counts as not armed. Without this, no remote could ever take control after a boot.
- The takeover check runs before the zero-proof counter update. Without this, a squeeze could never grant control.
- The paired check keeps the `paired` flag, and every unpair path clears the remote list.
- The takeover limit is published before the new owner's throttle, its start time is never zero, and there is no takeover before the motor output runs.
- Release zeroes throttle first, restores 6-byte receive mode, and clears the previous rider's stop notices.
- A spare remote auto-locks when not in control. The take-back is faster (about 1.6 s), and the display cycle is suppressed while it is armed.
- An RSSI gate on both ends while pairing.
- No old-style remote while two remotes are paired, and no "pair as legacy" command.
- Slow-mode collision hop of 33/66 ms only; no urgent buzz while slow.
- A bounded telemetry wait; the hop seed comes from the chip ID.

---

## 16. Open questions for the maintainer

1. **The takeover limit is a timed cap on manual throttle:** 25 % rising to full over 4 s, at most 10 s, only lowering, never GPS-based, multi-remote only. Is that an acceptable exception to "automation never leaves a cap on manual throttle"?
2. **"Another remote is in control"** as a legitimate reason a trigger does nothing (E9), next to the deadman, the link-loss failsafe and lock?
3. **Control passing to another remote** after about 6 s of the owner's silence ends Follow-Me / auto-return. That is a new way for auto-return to end, besides arriving, a fault, or switching the remote off and on. Acceptable?
4. **Older multi-remote notes in this fork are superseded:** a master timeout of 10-15 s, a guest never above the master's last throttle, all remotes on the same firmware. They are replaced by the three release rules, the takeover limit, and "new-style remotes only once two or more are paired". Agree?
5. **Board identities now come from the chip and cannot be changed.** They are not confidential (every packet carries them); buggies are kept apart by the link ID. Acceptable?

Feedback on the numbers is welcome too: the -60 dBm pairing gate, the 120 s spare auto-lock, the 1.2 s take-back hold, and a 1 s idle rate everywhere versus 500 ms where there is no duty limit.
