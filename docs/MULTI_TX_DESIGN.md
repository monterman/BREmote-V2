# Multi-remote design v3.2 ("hybrid")

> **Design v3.2: implemented (receiver SW37, remote SW29) and host-tested. NOT tested on hardware or
> water. ALPHA, bench only.**
>
> The code follows this design and passes the host tests in section 14.1. No part of it has run on a
> buggy or on the water yet. The rider-facing guide (pairing, screens, gestures, settings) is
> [`MULTI_TX_GUIDE.md`](MULTI_TX_GUIDE.md). Comments are welcome in the forum thread "BREmote - Future
> Development".

> **Before you test anything**
> - **This is a bench-test version.** Read [`MULTI_TX_GUIDE.md`](MULTI_TX_GUIDE.md) first: it has the pairing steps, the screens and the safety rules. Section 14 here is the maintainer's test plan, not flashing instructions.
> - **Back up every board first** (the Base64 line from `?conf`). It is the way back to the old firmware.
> - **Today's firmware, already:** a receiver obeys any remote addressed to its stored `own_address`. If a configuration was ever copied between receivers, two buggies can answer one remote. Check that no two receivers show the same `own_address` in `?conf`, and never power two buggies during any multi-remote test. (On SW37 the address is the chip ID, and a copied configuration drops its pairing at boot; a buggy still on the old firmware is not protected.)
> - `no_lock = 1` also turns off the spare-remote auto-lock, and with it set the LEFT hold does not lock a remote either. With it set, switch off every remote you are not holding.
> - Pair the remote you ride with **first**, at arm's length, one buggy at a time, and check the LED count and `?mtx` afterwards. The -60 dBm gate and "about 5 m" are calculated, not yet measured on hardware.
> - Bench with props off and the drivetrain secured.

**Base:** V2.5-Evo at commit `652112a`: receiver SW36, `confStruct` 200 bytes; remote SW27, 136 bytes. Line references below are at that commit.

**Versions (implemented):** receiver SW37 (232 bytes) and remote SW29 (144 bytes). Both carry the old settings over (section 13). v3.2 adds no new setting to v3.1. Section 15 ("From v3.2 to the implementation") lists what the code settled that this design left open or stated differently.

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

A design review before code produced the v3.1 changes. The maintainer's decisions of 2026-10-20 produced v3.2: the **claim model**, the **summon**, and the rule for **remotes without GPS** (section 15).

---

## 1. Scope

- **One buggy and up to two remotes.** The data structures keep room for four remotes, so a third remote later needs no layout change.
- **Several buggies near each other are not a supported use yet.** The packet format already keeps buggies apart (section 3), and a host test checks it. A separate design pass will cover that case.
- **With a single remote paired, nothing changes.** The receiver runs today's code path, byte for byte, **including what Follow-Me and Return-To-Me do when the remote goes silent** (section 5.6). The one visible change is on the remote's screen: a gesture that cannot engage now shows "St" (section 8.5).
- **Remotes with and without GPS can share a buggy.** A remote without a GPS fix gets manual control only (section 8.7).

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

**How control passes: the claim model.**
- **No timer ever releases the remote in control, and no timer ever ends Follow-Me or Return-To-Me.**
- **Lock** releases at once.
- With another unlocked remote on, the remote in control becomes **claimable**:
  - after **10 s of silence** (switched off or out of range; a remote that goes to sleep switches Follow-Me and Return-To-Me off itself first); or
  - after **10 s at zero throttle**, but only while Follow-Me and Return-To-Me are off.
- **Claimable changes nothing on the buggy.** Follow-Me or Return-To-Me stays armed, the motor failsafe keeps the motors stopped, and if the remote in control is heard again first, nothing has changed (a remote that was switched off and on still ends Follow-Me and Return-To-Me through its boot ID, as today).
- **Control passes only when the other remote claims it on purpose:**
  - a **squeeze from zero throttle**: manual control, under the takeover limit; or
  - a **summon**: its Return-To-Me or Follow-Me gesture, which needs its own GPS fix. It takes control, and the buggy then comes to that rider, through the normal arming steps.
- **A spare remote that nobody touches for 120 s locks itself.** A locked remote cannot claim and does not count as "another remote on".

**Takeover limit:** every time control passes, the new remote's throttle is limited to 25 % at that moment, rising evenly to full over 4 s, because the buggy may be near the other rider. Then it ends on its own.

**Master take-back:**
- throttle at zero;
- toggle RIGHT tap, then RIGHT hold 1.2 s;
- refused while Follow-Me or Return-To-Me is armed.
- About 1.6 s to motor zero, and it works even while the guest is driving. **It zeroes the guest's throttle at once (the buggy stops), so the master should not take back while the guest depends on the buggy.** The guest's remote shows "tA".

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

**Added by v3.2 (implemented in receiver SW37 and remote SW29):**

| Byte 3 | Byte 4 | Meaning | Sent by |
|---|---|---|---|
| `0xF1` | **`0x10`** | **LOCKED**: this remote is locked | new-style remotes only, always with a link-ID address |
| `0xF1` | **`0x5A`** | **SEIZE**: the master takes control back | the master only, same rule |
| `0xF1` | **`0x3C`** | **SUMMON**: a remote that is not in control claims a free or claimable buggy without a squeeze (section 5.5) | a remote that is not in control and has a fresh GPS fix, same rule; never while locked |
| `0xF5-0xFF` | - | still never sent; a v3.2 receiver **drops** them from every remote | - |

With a single remote paired, a receiver drops SEIZE and SUMMON.

**Why LOCKED, SEIZE and SUMMON live inside 0xF1.** Any new code must be harmless on a buggy that never got the update.

| Receiver build | What `0xF1/0x10`, `0xF1/0x5A` or `0xF1/0x3C` does there |
|---|---|
| before 2026-04-25, and upstream receivers | throttle 241. This is the same as every existing 0xF1-0xF4 meta packet on those builds; only the address protects them, and a link-ID frame never matches it. |
| 2026-04-25 to 2026-09-28 | the value is ignored, but on these builds any meta packet refreshes the motor failsafe |
| 2026-09-28 (`f79a50f`) to today | ignored; a liveness stamp that cannot keep the motor running |

A new code in `0xF5`/`0xF6` would instead be 96 % throttle on every one of these builds.

**One frame builder.** On the remote, a single function produces address and byte 3 together. It is the only place that can emit LOCKED, SEIZE or SUMMON, and only with a link-ID address.

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
| 010 | FREE | nobody controls, **or the remote in control can be claimed**: take it from zero throttle, or summon it |
| 011 | BUSY | another remote controls |
| 100 | SEIZABLE | a guest controls; you are the master and may take it back |
| 101 | REFUSED | another remote controls and your input is ignored → **E9** |
| 110 | SEIZED | the master took control from you, or another remote claimed it → **"tA"** |
| 111 | reserved | read as BUSY |

**Each remote gets its own telemetry rotation.** With one shared rotation, a remote answered every 10th time would never see some indices.

**A remote that is not in control** gets the owner's Follow-Me and Return-To-Me fields blanked, so a guest's screen never shows someone else's mode. Battery, temperature, speed, water alarm and its own link quality are sent normally.

### 4.4 Pairing frames (byte 0)

| Frame | Today | v3.2 |
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
| SINGLE | exactly one remote paired (old or new) | today's code path. A LOCKED frame is handled exactly like today's locked packet (0 / 127). SEIZE and SUMMON are dropped. Status always 000. No takeover limit (except once, right after a list change from MULTI, section 5.3), no claim. |
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

**Control packet from another remote, while the owner is NOT claimable:**
- with input → REFUSED (E9);
- without input → BUSY, SEIZABLE or SEIZED.
- The owner's motors are not affected.

**Control packet from another remote, while the owner IS claimable** (section 5.4):
- zero proof (as above) and a squeeze → **a claim** (section 5.5);
- a held trigger without zero proof → FREE (no E9, no claim).

**LOCKED from the owner:** release at once. A locked frame never counts as zero proof, so an unlock squeeze cannot become a takeover.

**SEIZE from the master:** accepted when all of these hold:
- the owner is a guest;
- take-back is enabled;
- Follow-Me and Return-To-Me are not armed;
- the master sent zero throttle within the last second.

It is accepted even while the guest is driving.

**SUMMON from another remote:** accepted when all of these hold:
- nobody owns, or the owner is claimable;
- the summoning remote sent zero throttle within the last second;
- the motor output is running.

Otherwise it is dropped. The summoning remote, which gets no "you control" answer, shows "E9" if its latest status says another remote is in control, else "St" (section 8.5).

**Meta packets** (RTM, Follow-Me, GPS, aux): from the owner only. A guest's GPS announce never switches the receiver into 15-byte mode.

### 5.3 Release and takeover

**Release** happens on a lock, a master take-back or a claim, never on a timer. It does this, in this order:
1. throttle 0 and steering centre (no failsafe stamp, so the gate shuts on its own clock);
2. "RTM off" through the existing handler (which arms the slow ceiling of a return that was running; it only lowers throttle, and because step 1 already zeroed the throttle it normally clears on the next motor pass);
3. "Follow-Me off";
4. the rider-GPS state, the GPS cross-checks and the remote boot ID reset;
5. an owner-change counter.

The receiver's return-to-me loop watches that counter. It closes the short "how the last run ended" windows for both Follow-Me and return-to-me, so a new rider never sees the previous rider's stop on his screen.

**A change of the paired list while running** (a configuration import, `?applyconf`, `?set paired 0`) releases the owner with the same order, and the receiver starts again from "nobody owns". Whenever the list was MULTI before the change or is MULTI after it, step 1 (throttle 0, steering centre, no failsafe stamp) runs even with nobody in control. A change from one remote to one remote stays today's behaviour. **From MULTI to SINGLE** (two or more remotes to one) the takeover limit is also published once, after the zero and before that wake's frame: the board is SINGLE from then on, so the remaining remote's next control frame is applied at once, without zero proof, and its first byte meets the limit (section 6) and the motor ramp. Both rules are the pure `mtxListChangeActions()` (host test T21).

**Takeover:**
- The **takeover limit** (section 6) is published first, then the new owner's throttle.
- The limit's start time is never stored as 0, because 0 means "no limit".

### 5.4 Claimable: the claim timers

- **"Another remote on"** = a remote other than the owner sent a control packet (not LOCKED) within the last 3 s.
- **Silent:** nothing heard from the owner for 10 s in total, and never less than the failsafe time + 1 s.
- **Idle:** the owner has shown no activity for 10 s, its last throttle byte is below 8, **and Follow-Me and Return-To-Me are not armed.** While either is armed, idle never counts; only silence does, and idle needs a full 10 s at zero after the disarm.
  - **Activity** = a control frame with throttle 8 or more or steering 24 or more off centre, an RTM state frame `0xF1/0` or `0xF1/1`, or a Follow-Me declaration made by a rider gesture (`0xF2` with bit 7). The 30 s keepalive, the boot ID, the refresh, aux and the GPS announce are not activity.
  - **The remote in control stays busy on any button.** While a toggle press or hold, a magnet contact or the RTM arm ceremony is in progress (and for 3 s after the last press), the remote in control repeats `0xF1/0` at most every 2 s, so a rider working through a gesture at zero throttle is never idle. It does this only with status OWNER (001), never with one remote, never while RTM is active and never with an `0xF1/1` queued.
- **Silence and idle together:** a silence of `mtx_claim_s` (10 s) or more restarts the idle clock when the owner is heard again, so an owner that comes back from a long silence is never idle-claimable on its first packet. A shorter silence still counts as time at zero: 5 s at zero followed by 5 s unheard makes the owner idle-claimable at 10 s, before it would be silent.
- **Claimable** = another remote on, and the owner silent or idle.
- **Claimable releases nothing and ends nothing.** The other remotes hear FREE. The owner keeps control until another remote claims it.
- **The owner coming back** (any packet after silence, any input after idle) ends claimable at once. It never lost control, so it gets no limit.
- **Without another remote on, the owner keeps control**, exactly as today.
- One setting, `mtx_claim_s` (10 s), serves both the silence and the idle time.
- **Motor safety never depends on these timers.** The existing motor failsafe shuts at failsafe time on its own.

### 5.5 Claims and the summon

**A claim by squeeze** (from a claimable owner, or after a lock or a boot when nobody owns):
- It needs zero throttle first, then a squeeze, like any takeover.
- The receiver releases the old owner (section 5.3), then grants the new one under the takeover limit.
- If Follow-Me was running on the buggy, it takes up to one Follow-Me tick (100 ms) to go idle. For up to 1 s, the claim is held for the claiming remote only. Nobody else can take control in that time.

**A summon:** the remote that is not in control does its normal Return-To-Me gesture (RIGHT tap, then LEFT hold) or its Follow-Me gesture (LEFT tap, then RIGHT hold, or the magnet), with the trigger released.
- **On the remote, first:** the summon is refused and nothing is sent, unless:
  - the latest status is FREE (and at most 5 s old);
  - the remote has a fresh GPS fix (the same test that lets it send its rider-GPS stream);
  - the trigger is released;
  - the feature is enabled;
  - the remote is not locked.

  The refusal shows **"E9"** when the latest status says another remote is in control and not claimable (BUSY, SEIZABLE, REFUSED) or has just taken control from this one (SEIZED), the same answer a squeeze gets. Every other refusal (no fresh fix, the feature off, the trigger held, a status older than 5 s) shows **"St"** and the stop buzz.
- **The wait:** after the burst the remote waits up to 1.5 s for an owner status, without blocking. It judges the arrival time of the **first** owner status after the send, so the answer is the same however late it is read; it is read on every loop pass and also inside the toggle handler, which blocks the loop while the button is held. None in time: "E9" if the status now says another remote is in control, else "St". If the grant arrives later than that, the remote is in control with nothing armed (its dot goes solid) and the rider makes the gesture again.
- **On the buggy:** it is accepted only if nobody owns or the owner is claimable, and the summoning remote was at zero within the last second. **A summon can never take control from a remote that is commanding the buggy.**
- **Then the summoning remote is the owner** (under the takeover limit), and it arms Return-To-Me or Follow-Me **through today's normal arming path.** The buggy checks every gate as it does today:
  - the new rider's GPS stream (only the owner's stream is accepted);
  - the GPS cross-check, including the maximum rider distance (it restarts with each owner, so it runs on the first GPS packet from the new rider);
  - its own GPS and heading.
- **The previous rider's fix is marked stale at the release:** its time stamp is cleared. Every check that reads the rider's position requires a fresh time stamp, and the three rider-position filters start over, so the buggy can never steer toward where the previous rider was. (The coordinates themselves stay in memory, unused.)
- **The arm re-checks before it runs** (it waits up to 600 ms for the new owner's boot ID to go out): still in control, the trigger released, no return running or arming on this remote, and for Follow-Me not already armed. Otherwise nothing is armed. An arm the rider makes himself on the owner path in the meantime cancels the summon.
- Nothing moves without the summoning rider's trigger: Return-To-Me still needs its squeeze ceremony, and auto-return needs the squeeze.
- If the buggy then refuses the arm (for example the rider is beyond the maximum distance), the summoning remote keeps manual control and sees today's "St" or not-ready screen.

**The rider whose control was claimed** sees "tA" when his remote is heard again, and his remote drops its Follow-Me screen.

### 5.6 Single remote: nothing changes, including when the remote goes silent

With one remote paired, or with the second remote switched off or locked, **no claim can occur**: a claim needs a second remote that is on, unlocked and transmitting. With the second remote off or locked, two things still differ from a single paired remote, and neither touches Follow-Me or Return-To-Me:
- after the remote's own lock (which releases control) or a buggy reboot, nobody owns, so the remote's next squeeze takes control under the takeover limit;
- in that state its Return-To-Me or Follow-Me gesture is a summon. Nothing is armed while nobody owns, so a summon there ends nothing; the arm that follows is the normal one.

So the buggy does exactly what today's firmware does when its only remote goes silent:
- **Motor:** the gate shuts at failsafe time (default 1 s). When the remote is back, it reopens with the soft ramp from 0.
- **Return-To-Me:**
  - trigger held when the link went: the emergency stop at once, then RTM ends after 1.5 s with "St";
  - trigger released when the link went: RTM stays armed until the remote comes back, or until the remote reboots.
- **Follow-Me, armed or following:** a pause; after 30 s without the link (or either GPS fix), Follow-Me ends with "St".
- **Parked auto-return:** it waits ("not ready") through the silence, until the declaration expires 95 s after the last Follow-Me declaration packet, which is 65-95 s after the remote went silent (the keepalive is every 30 s).
- **A remote that reboots** ends Follow-Me and RTM through its boot ID.

v3.2 neither shortens nor extends any of these.

---

## 6. Motor path: the takeover limit

- One `min()` in the receiver's motor calculation, after the existing caps and before the output ramp: `throttle = min(throttle, limit)`.
- The limit is 64 of 255 (25 %) at takeover, rising evenly to full over 4 s: 64 / 111 / 159 / 207 / 255 at 0 / 1 / 2 / 3 / 4 s.
- **It applies only at the moment control passes** (a takeover, a claim, a summon or a take-back), **because the buggy may be near the other rider when it does.** Then it ends on its own. A remote that never lost control (for example one coming back after a silence) gets no limit.
- Both numbers are settings. The time can never exceed 10 s.
- It only lowers throttle, never reads GPS, and is otherwise never used with a single remote. **The one exception:** once, right after the paired list changes from two or more remotes to one (section 5.3), so the remaining remote restarts from zero under the same soft start.
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

**The BIND LED:**
- slow blink while the window is open;
- N flashes right after each remote pairs (N = its slot; 1 = the master);
- at close, solid for 1 s = the new list is saved;
- three fast blinks = nobody paired and the old pairing is kept, or the radio refused the pairing power and the window did not open.

**At close:**
- With at least one remote paired, the new list (in pairing order; first = master) is saved once.
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

**An old-style remote is the exception.** The budget above assumes both ends at -9 dBm. An old-style remote sends its pairing request at its stored RF power (22 dBm by default), about 31 dB more, so the receiver's -60 dBm gate admits it from tens of metres or more. Its pairing still closes the window at once (it can only be the single remote), but pair it at arm's length anyway, and test the 15 m refusal only with a new-style remote.

**Pair the remote you ride with first,** so it becomes the master.

---

## 8. The remote

### 8.1 Rate

- **The owner** sends every 100 ms, as today, with the collision backoff unchanged.
- **A remote that is not in control** drops to one packet per second when idle.
  - It stays slow even with input while another remote is in control and cannot be claimed. E9 still shows within about a second.
  - It goes fast:
    - when control is free or claimable and it is squeezed or a gesture is started;
    - at the first press of the take-back gesture;
    - when it has had no reply for 5 s.
- In slow mode a missed reply shifts the next send by 33 or 66 ms, never 0, so it cannot sit in the owner's slot.
- While slow, the "connected" window is 2.5 s instead of 1 s, and the urgent link-lost buzz is suppressed (the blink still shows).

### 8.2 Owner-only actions, and the summon

- Follow-Me arm and mode changes, Return-To-Me arm, aux, and the rider-GPS stream need ownership.
- **A remote that is not in control:** a Return-To-Me or Follow-Me arm gesture becomes a **summon** (section 5.5) when the status is FREE and the remote has a fresh GPS fix. Otherwise it sends nothing and shows "E9" (another remote is in control) or "St" with the stop buzz (section 5.5).
- **A toggle tap on a remote that is not in control changes nothing:** no gear, no cap.

### 8.3 Lock and auto-lock

- A new-style remote sends LOCKED while it is locked.
- **Auto-lock for a remote that is not in control** (in multi-remote mode): 120 s by default (setting, 0 = off).
- **Auto-lock for a single remote, or the owner:** off by default (setting).
- Auto-lock is skipped while Follow-Me or Return-To-Me is armed, during the take-back gesture, in a menu or in setup, and with `no_lock = 1`.

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

**On completion** the remote sends SEIZE three times. The receiver re-checks every condition itself. An owner status that first arrives within 1 s of the send gives the arm-confirm buzz (two quick taps); none gives "St". The answer is read while RIGHT is still held (the toggle handler blocks the loop), and it is judged on that first arrival, so holding longer never turns a success into "St".

**About 1.6 s from the first press to motor zero.** An accidental take-back can only slow the guest's ride to a stop; it can never start the buggy. **It works while the guest is driving, and it zeroes the guest's throttle at once (the buggy stops), so the master should not take back while the guest depends on the buggy.** It is refused while Follow-Me or Return-To-Me is armed. It is the master's only, and it can be switched off (`mtx_seize_en`).

### 8.5 Screens

- **E9**: refused. One warning buzz (two short pulses) on the rising edge; it shows for 1.5 s after the last REFUSED. This is not the remote's error state, so the next toggle press is not taken as an error acknowledgement. It shows only while the remote in control cannot be claimed. A summon refused because another remote is in control also shows E9.
- **"tA"**: control was taken (a take-back or a claim). Shown 3 s, with the same two short pulses. A remote that was silent when its control was claimed shows it when it is heard again.
- **"St"** with the stop buzz: no new glyph. With two remotes: a summon refused for any other reason (no fresh fix, the feature off, the trigger held, a stale status; held without blocking whenever the trigger is held) or not answered within 1.5 s; a take-back not confirmed within 1 s; and a remote that lost control with Follow-Me or RTM armed but without a "tA" (for example a receiver reboot).
- **"St" for every remote, one remote included** (the maintainer's rule: "if it cannot engage, show stop"): a Follow-Me or RTM gesture that cannot engage (the feature off, or the trigger held where it must be released) now shows "St" instead of nothing; held 2 s without blocking `loop()` when the trigger is held. While this remote's own Return-To-Me is running or arming, or its own Follow-Me is armed, the refusal is a brief 1 s "St" that never blocks, then the running mode's screen comes back. The RTM arm ceremony's two timeouts (no first or no second squeeze within `rtm_arm_window_s`) also show "St", held 2 s. **No "St" for a blocked end or change:** a gesture that would only have ended or changed a running mode (a magnet RTM hold during a return, the Follow-Me combo while Follow-Me is armed) and is blocked by the held trigger shows nothing new; the mode continues and its screen stays, as in SW27. A remote that is not in control and makes a toggle gesture with the trigger held takes the summon gate, as the magnet does: E9 against a busy owner, else "St".
- **Ownership dot:** shares the Follow-Me status dot (C7 R2) and shows only while the Follow-Me dots are off. SOLID = in control (OWNER); PULSE (on-off-on-pause, 1.2 s) = FREE; SLOW (a 200 ms blip every 3 s) = BUSY, SEIZABLE, REFUSED or SEIZED; OFF = one remote (OWNER_SOLO), an old-style link, or no reply for 5 s. `?mtxdot [0-3|auto]` forces it for the display check.

### 8.6 Smaller fixes on the remote

- The telemetry wait keeps listening after a frame meant for someone else, until its own reply or 40 ms after the send. It never touches the radio after that bound.
- The random seed of the collision hop comes from the chip ID, so two remotes with copied configurations do not hop in step.

### 8.7 Remotes without GPS

**A remote without GPS, or without a fresh fix right now, gets manual control only.**
- It takes control only by squeezing from zero (when the buggy is free or the remote in control is claimable), under the takeover limit.
- **It cannot summon.** The remote refuses with "St" and sends nothing.
- **A remote that has a GPS module but no fix counts as having no GPS until it gets one.**

**How the buggy knows.** It needs no setting and no capability bit: the rider-GPS stream is the answer.
- A remote sends its stream only while it has a valid fix that is not stale.
- Every Follow-Me and Return-To-Me check on the buggy already needs that stream to be fresh.
- A capability bit would say "has a module", which is the wrong question: the module may have no fix, lose it, or be unplugged.

**If the remote in control loses its fix during Follow-Me,** today's behaviour applies, with one remote or two:
- Follow-Me pauses and shows not-ready;
- after 30 s without the fix it ends with "St";
- a parked auto-return waits, and a squeeze while not ready gives "St".

Losing the fix never makes the remote claimable: it is still transmitting.

**Two typical cases:**
- **The GPS remote is in Follow-Me and goes silent for 10 s.** The remote without GPS can take over only by squeezing from zero. That ends Follow-Me on the buggy, and it drives manually.
- **The remote without GPS is in control, and the GPS master wants Follow-Me.**
  - The master takes back (this stops the buggy at once), then arms Follow-Me normally.
  - Or, once the other remote has been at zero (or silent) for 10 s, the master summons with its Follow-Me gesture.

---

## 9. Old and new mixed

| Buggy | Remote | Result |
|---|---|---|
| old | old | today |
| old | new, with its migrated legacy link | works as today (today's frames; locked = 0 / 127) |
| old | new, tries to pair | no pairing ("EP"); restore the remote's backup to go back |
| new (migrated) | old, existing pairing | works as today, **if** the buggy's stored address equals its chip ID; otherwise pair again |
| new | old, paired in a new window | accepted as the single remote |
| new | new | full v3.2 |
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
| Owner goes silent (switched off, out of range) | the motor failsafe at failsafe time after its last control packet; other remotes' packets cannot refresh it | **never by a timer.** After 10 s of silence, and only with another unlocked remote on, the owner is claimable; Follow-Me / Return-To-Me stay as they are today; control passes only on a squeeze from zero (under the limit) or a summon (needs a GPS fix) |
| Owner comes back before any claim | resumes from 0 with the soft ramp | - (still owner, no limit, Follow-Me / Return-To-Me unchanged) |
| Owner locks | throttle 0 at the first LOCKED frame (after the lock flush, about 300 ms) | at that frame, from zero, under the limit |
| Owner at zero 10 s, another remote on, no FM/RTM armed | already at zero | the owner is claimable; its own squeeze is still its own; the other remote claims from zero under the limit, or summons |
| Owner at zero with FM/RTM armed, however long | - | **never** (idle does not count while armed) |
| Summon against a remote that is commanding | nothing; the summoning remote shows "E9" (its status says another remote is in control, so it sends nothing; a summon that does reach the buggy is dropped) | never |
| Summon from a remote without a fresh GPS fix | - | refused on that remote ("St"), nothing sent; it may still claim by squeezing |
| Summon accepted, then the buggy refuses the arm (its own GPS, the rider too far away) | - | the summoning remote keeps manual control, from zero, under the limit; the previous Follow-Me had already ended |
| Receiver reboots with several remotes paired | no owner, gate shut | the first remote with zero proof and a squeeze; **a held trigger never restarts the buggy** |
| A guest squeezes while the master rides | nothing; the guest sees E9 | never from an active master |
| Master take-back | output to 0 at once, also while the guest is driving | master at zero + gesture + FM/RTM off |
| Control changes while FM/RTM was armed (a claim after 10 s of silence) | FM/RTM end on the buggy; rider GPS and checks reset; the old rider's stop notices are cleared | after Follow-Me is idle (≤ 100 ms; the claim is held for the claiming remote for up to 1 s) |
| The rider whose control was claimed comes back | - | he sees "tA"; he can come back only by the same rules |
| The owner's GPS fix is lost during Follow-Me | today's pause, then "St" after 30 s | never because of the fix (the owner is still transmitting) |
| **Unattended spare remote** | locks itself after 120 s; a locked remote is not "on", so the owner is never claimable and the spare cannot take control | in the first 120 s after it was last touched: an owner silent 10 s, or at zero 10 s with nothing armed, is claimable, and a squeeze on the spare takes control under the limit. The master's take-back stops it in about 1.6 s if nothing is armed. **If the rider's own remote is the guest, it cannot stop a spare that stays squeezed**, which is why the rider's remote is paired first. |
| A stranger's remote during the pairing window | the RSSI gate (≥ -60 dBm, about 5 m) on both ends; LED count and `?mtx` list after the window | a stranger within about 5 m during the window remains possible; pair one buggy at a time |
| Unpaired buggy | nothing accepted | - |
| Unpair paths (binding reset on import, `paired = 0`, factory reset) | the remote list is cleared with them | - |
| A copied or edited buggy configuration | identity from the chip; the copied pairing is dropped | - |
| Corrupt reply | the buggy decides; a remote needs 3 non-owner replies in a row before it slows down or disarms | - |
| Accepted, unchanged from earlier notes | if the master's remote fails **while still transmitting** (an undetected stuck input), a guest cannot take over | only once it goes silent (then a claim) or the input fault fires |

---

## 11. Interactions

**Rider GPS:**
- Only the owner's stream is sent and accepted.
- On a change of owner the GPS state and cross-checks reset. A new owner's first GPS packet runs the cross-check at once.
- The existing GPS integrity rules hold: every fix is judged, the 0xF3 sequence resets on any mismatch, int32 microdegrees are kept, and nothing is extrapolated.

**Follow-Me declarations** are per owner. A new owner arms with its own gesture (a summon runs that gesture for it).

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
- A summon costs one burst of three 6-byte packets. The remote is fast for the gesture plus 3 s, then it is the owner and the rows above apply. No row changes.
- **US 902-928 MHz:** no duty limit. A fixed 250 kHz channel does not fit FCC 15.247 as it stands (lower power or duty, or FHSS, or a 500 kHz preset). This design neither fixes nor worsens that.

---

## 13. Configuration

**Receiver, SW36 → SW37, 200 → 232 bytes (appended):**

| Field | Default | Range | Purpose |
|---|---|---|---|
| `mtx_id[12]`, `mtx_flags[4]` | 0 | written by pairing only | up to 4 remote IDs in pairing order (slot 1 = master); per slot: valid, legacy |
| `mtx_claim_s` | **10** | 6-60 s | with another remote on: the silence, or the time at zero with Follow-Me / Return-To-Me off, after which another remote may claim control (by a squeeze from zero or a summon). The silence is never shorter than failsafe time + 1 s. |
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
- **Backups:** a Base64 backup (`?conf`) carries everything, the remote list included, and restores the list only on the board it was made on. A JSON backup never carries the list: restoring one keeps the board's current list. A JSON made on **another** receiver imports its settings but not its pairing (the receiver keeps its own). `mtx_link_proto` is set by pairing only and is skipped in a JSON import.
- **No setting for the GPS rule.** It is decided at run time from the rider-GPS stream (section 8.7).
- New commands:
  - `?mtx` on both boards: the list, roles, owner, status, rates, counters, whether the owner is claimable (and why), and the last refused summon;
  - `?mtxdot` on the remote (dot test).
- Every new field goes into both web pages and the web serial config tool.
- `TelemetryPacket`, the logging struct, the packet size and the radio settings do not change.

---

## 14. Tests before any flash, then the bench

### 14.1 Host tests (`Tools/tests/multi_tx_test.cpp`, against a pure `Source/Common/MultiTx.h`)

| # | Covers |
|---|---|
| T1 | link ID round trip; never zero; uplink ≠ reply address |
| T2 | classification on two buggy IDs; a frame for one buggy is unknown on the other (this is the only coverage of the multi-buggy case for now) |
| T3 | byte-3 decode, including LOCKED / SEIZE / SUMMON exactly, 0xF5-0xFF dropped, SEIZE and SUMMON dropped with a single remote |
| T4 | takeover: zero, zero, squeeze → yes; held trigger → never; FM armed → no; FM never declared → allowed |
| T5 | take-back conditions |
| T6 | (see T31) |
| T7 | (replaced by T23-T25 and T33: no timer may release) |
| T8 | the takeover limit curve and its edge cases |
| T9 | status packing; status always 000 with one remote |
| T10 | the telemetry masks |
| T11 | per-remote telemetry rotation |
| T12 | entry and binding validation |
| T13 | the pairing window, including the RSSI gate |
| T14 | old/new mixes, including the SUMMON code on old receiver builds |
| T15 | the remote's single frame builder (no LOCKED / SEIZE / SUMMON outside a link-ID address, nothing ≥ 0xF5) |
| T16 | remote status, rate, auto-lock, take-back and summon logic; the take-back answer judged on the first owner arrival, however late it is read |
| T17 | link-fresh windows |
| T18 | the bounded telemetry wait |
| T19 | configuration migration on real configuration exports of each board. These are kept locally because they contain pairing data, and are not committed. |
| T20 | the paired check and every unpair path |
| T21 | the release order (including a claim) and the clearing of the previous rider's stop notices; what a paired-list change does (zero with MULTI on either side, the limit once on MULTI -> SINGLE) |
| **T22** | **single remote, and a second remote off or locked: packet for packet, the throttle, failsafe, Follow-Me and Return-To-Me writes equal today's reference through silences of 0.5-100 s; no claim, no release**. Also, with the second remote off: lock → nobody owns → a squeeze (takeover limit) or a summon (no Follow-Me / Return-To-Me write before the new owner's own arm) |
| T23 | claimable but not released: silent 9.999 s → no, 10 s → yes; still the same owner after 100 s; failsafe time 10 s → 11 s |
| T24 | the owner returns before a claim: still owner, no limit, nothing changed |
| T25 | the idle rule: 10 s at zero → claimable; never while Follow-Me / Return-To-Me is armed; input restarts it |
| T26 | a claim by squeeze: release order, the 1 s hold for the claiming remote, its expiry, nobody else granted meanwhile; a held trigger never claims |
| T27 | summon gates on the buggy: free or claimable only; zero within 1 s; motor output running |
| T28 | summon refused against a remote that is commanding (armed and transmitting, or driving), or silent under 10 s |
| T29 | the remote's summon gate: FREE status, a fresh GPS fix (no module, no fix, stale fix → refused, nothing sent), trigger released, not locked; the answer judged on the first owner arrival (a late grant stays late) |
| T30 | rider GPS after a claim: the previous rider's position wiped; the cross-check (including the maximum distance) runs on the new rider's first packet |
| T31 | status priority: taken > busy (control passing to someone else) > free / claimable > refused > take-back offered > busy; "tA" for a remote whose control was claimed while it was silent |
| T32 | the SUMMON frame: decode, builder, never queued as a merged RTM packet |
| T33 | claim thresholds across the full setting ranges and the `millis()` wrap |
| T34 | the remote's activity repeat: from the first press, at most every 2 s, owner status 001 only, never in SINGLE, never with RTM active or an `0xF1/1` queued |
| T35 | a refused arm gesture: "E9" against a busy owner, "St" otherwise |
| T36 | the ownership dot, and the non-owner test the summon path uses |
| T37 | the remote's config rules: the link protocol (0 or 3, otherwise unpaired) and the three clamps |

### 14.2 Builds

- **Receiver:** `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=custom` (it uses its own `partitions.csv`; never `huge_app`).
- **Remote:** `esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=huge_app`.
- **The classic remote and the Nano remote build from the same sketch, the same FQBN and the same binary.** Their differences (GPS chip, calibration, magnet sensor) are configuration.
- Zero errors, zero new warnings.

### 14.3 Bench (props off, motors secured, every other receiver switched off)

**Phase A: receiver updated, remotes still old.**
- migration and identity notes;
- old-style pairing as the single remote (at arm's length: an old remote pairs at its normal RF power, so the receiver's RSSI line is read at 1 m only - the 15 m refusal is a Phase B test);
- the single-remote regression (identical to today);
- an aborted window keeps the pairing and restores the RF power.

**Phase B: first remote updated.**
- remote migration, with no calibration prompt;
- a locked legacy-link remote still sends 0 / 127;
- new-style pairing: refused at 15 m, accepted at 1 m, RSSI recorded on both ends (both ends at -9 dBm: a new-style remote on both sides of the test).

**Phase C: both remotes updated, paired in one window (rider's remote first).**
- ownership and dots;
- E9;
- claimable by idle (manual only), with the presence rule; release by lock and by auto-lock;
- takeover from zero under the limit;
- a guest can never take from an active master;
- the master take-back (≤ 2 s), released at once and after holding RIGHT 3 s: the confirm buzz both times, never "St";
- the owner switched off mid-throttle: the failsafe shuts in ≤ 1.1 s, nothing from the other remote reopens it, the owner becomes claimable at 10 s and is **never released on its own**;
- Follow-Me armed and the owner silent: claimable but still armed; the owner back first changes nothing; then a claim by squeeze;
- summon by the Return-To-Me gesture and by the Follow-Me gesture (needs a GPS on the buggy and a fix on the summoning remote);
- a toggle summon held 2 s past the combo: it still arms;
- during a return, a magnet RTM hold with the trigger held: no "St", the return continues;
- a paired-list change from two remotes to one with the remaining trigger held: the output restarts from zero at about 25 %, full after 4 s;
- summon refused: against a remote that is driving, against an armed remote that is transmitting, after only 8 s of silence, from a locked remote;
- **a remote without GPS:** set one remote to `gps_en 0`, and separately use a remote whose module has no fix yet (indoors). Its summon is refused with "St" and nothing is sent; it can still claim by squeezing; the GPS master takes back, then arms Follow-Me;
- **single-remote silence:** with only one remote paired, and again with the second remote off or locked, Follow-Me and Return-To-Me behave on a silence exactly as today (pause, the 30 s end, the parked wait), with no claim line at all;
- the idle rate over 10 min, with no urgent buzz;
- airtime;
- the unattended spare;
- an owner change right after a Follow-Me fault stop;
- the unpair paths.

**If both your remotes have GPS,** that is the usual case. The no-GPS test above simulates the mixed case with `gps_en 0` (restore it afterwards).

**Then the same core cases on the main buggy**, and the bench receiver's list is cleared.

---

## 15. What changed

### From v3.2 to the implementation (receiver SW37, remote SW29)

Settled while the code was written, by the maintainer or by the code review. The sections above already say it this way.

- **The refusal signal.** A summon refused because another remote is in control shows "E9", the same answer a squeeze gets; every other refusal shows "St" (sections 5.5, 8.5).
- **"St" when it cannot engage, on every remote.** A Follow-Me or RTM gesture that cannot engage, and the RTM arm ceremony's timeouts, show "St" instead of nothing, one remote included. Over a running return or this remote's armed Follow-Me it is a brief 1 s "St" (an interim choice, still to be confirmed); with the trigger held it does not block `loop()`. A blocked end or change of a running mode shows no "St" (section 8.5).
- **The take-back and summon answers** are judged on the first owner status after the send and read while the button is held, so a longer hold never reads as "no answer"; the summon's arm re-checks its preconditions first (sections 5.5, 8.4).
- **The remote in control is busy on any button** (the `0xF1/0` activity repeat, section 5.4).
- **A toggle tap on a remote that is not in control changes nothing** (section 8.2).
- **A change of the paired list while running** zeroes throttle and steering whenever two or more remotes are involved (section 5.3; confirmed by the maintainer 2026-10-21). From two or more remotes to one, the remaining remote also gets the takeover limit once ("start from zero and with a soft start").
- **A JSON made on another receiver keeps the receiver's own pairing** (section 13; confirmed by the maintainer 2026-10-21).
- **The pairing LED** flashes N times right after each enrolment, then shows solid 1 s when the list is saved (section 7).
- **Free storage before the first boot's writes:** the receiver saves the migrated configuration (and the web page) only with at least 1000 KB of SPIFFS free; otherwise it runs from RAM and a NOTE says what to free. Do not pair before freeing space: a pairing's save removes the stored file first, so both could be lost.

### From v3.1 to v3.2 (the maintainer's decisions, 2026-10-20)

- **No timer releases control any more.** The 5 s idle release and the "failsafe time + 5 s" link-loss release are gone.
- **The claim model:** after 10 s of silence, or 10 s at zero with nothing armed, and only with another unlocked remote on, the remote in control becomes claimable. Follow-Me and Return-To-Me stay armed until another remote claims on purpose (a squeeze from zero, or a summon).
  - The reason: with one remote, or with the other remote off, today's Follow-Me must not change at all.
  - With two remotes, Follow-Me may end only because the other rider consciously took the buggy.
- **The summon:** the other rider's Return-To-Me or Follow-Me gesture claims the buggy and brings it to him, with his own GPS. New code `0xF1/0x3C`.
- **A remote without a GPS fix gets manual control only.** It is decided at run time from the rider-GPS stream; there is no setting.
- **The owner pausing at zero no longer loses control.** It becomes claimable, and its own next squeeze is its own throttle, with no takeover limit.
- **The takeover limit applies only at the moment control passes** (the reason is now stated: the buggy may be near the other rider).
- **The master take-back is kept, also while the guest is driving.** It stops the buggy at once.
- `mtx_release_s` becomes `mtx_claim_s`, default 10 s. No new setting on either board.

### From v3 to v3.1

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

## 16. Decisions by the maintainer (2026-10-20), and what is still open

1. **The takeover limit** (25 % rising to full over 4 s, at most 10 s, only lowering, never GPS-based, multi-remote only) **is accepted** as an exception to "automation never leaves a cap on manual throttle". It applies only at the moment control passes.
2. **"Another remote is in control"** is accepted as a legitimate reason a trigger does nothing (E9), next to the deadman, the link-loss failsafe and lock.
3. **Follow-Me / auto-return ending with two remotes:** decided as the claim model above. Silence alone never ends Follow-Me; only a deliberate claim by the other remote does, after 10 s of silence. With one remote, nothing changes.
4. **The older multi-remote notes in this fork are superseded** (a master timeout of 10-15 s, a guest never above the master's last throttle). They are replaced by the claim model, the takeover limit, and "new-style remotes only once two or more are paired".
5. **Board identities come from the chip and cannot be changed.** They are not confidential; buggies are kept apart by the link ID. Accepted.

**Feedback on the numbers is still welcome:** the 10 s claim time, the -60 dBm pairing gate, the 120 s spare auto-lock, the 1.2 s take-back hold, and a 1 s idle rate everywhere versus 500 ms where there is no duty limit.
