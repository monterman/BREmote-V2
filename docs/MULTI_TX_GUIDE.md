# Multi-Remote: Rider's Guide

**BREmote V2.5-Evo** · one buggy, two remotes · receiver SW37, remote SW29

> **Status: ALPHA, bench-test version, not tested on hardware or water.**
> The code is written and host-tested only. Nothing in it has run on a buggy or on the water yet.
> Bench it with the props off before anything else (§12). The engineering design and the full test
> plan are in [`MULTI_TX_DESIGN.md`](MULTI_TX_DESIGN.md).

---

## 1. What it is

Two riders share one buggy, each with their own remote. **One remote at a time is in control**: only
its throttle and steering reach the motors. The other remote is ignored until control passes to it on
purpose.

- **One buggy, two remotes** is what this version is built and tested for. The buggy can list up to
  four remotes, but more than two has not been tested.
- **The master** is the remote paired **first**. It has one extra power: it can take control back from
  a guest (§6). Every other remote is a **guest**.
- **No timer ever hands control over.** Control passes only when the remote in control locks, when
  another remote claims it on purpose, or when the master takes it back.
- **With one remote paired, nothing changes.** The buggy runs today's code path. The one visible change
  on the remote is that a Follow-Me or Return-To-Me gesture that cannot engage now shows `St` (§4).

The rule from the Follow-Me guide holds for every remote:

> **The buggy moves only while the rider in control holds the throttle trigger. Letting go of the
> trigger stops it.**

---

## 2. Before you start

| You need | Why |
|---|---|
| **The buggy and every remote on this firmware** (receiver SW37, remote SW29, the same remote build on every remote) | A remote on older firmware can still pair, but only as the buggy's single remote, and it closes the pairing window at once. |
| **A backup of every board**, taken before you flash: the Base64 line from `?conf`, or the Base64 export in the web tool | It is your way back (§14). |
| **Free storage on the buggy**: download and delete old logs before flashing (`?list`, then `?deletelog <file>`) | The first boot saves the carried-over settings only with about 1000 KB free. With less, a NOTE line says so, nothing is saved, and a pairing made then is not kept either. |
| **Every remote paired again, all in one window** (§3) | The multi-remote link needs a new pairing, and each pairing window replaces the whole list. |
| **A GPS with a fix on every remote that should use Follow-Me, Return-To-Me or the summon** | A remote without GPS, or without a fresh fix right now, gets manual control only (§5). A GPS module with no fix counts as no GPS until it gets one. |

### What carries over when you flash

- **Buggy (SW36 to SW37):** every setting and the compass calibration are kept. The four new settings
  take their defaults (§11). The old pairing is kept only if the buggy's stored address is its own chip
  ID. If its configuration was ever copied from another board or edited, the first boot drops the
  pairing and prints a NOTE line. Either way, pair again for two remotes.
- **Remote (SW27 to SW29):** every setting and the throttle calibration are kept. The old pairing stays
  as an **old-style link**: it works as today, but only as the buggy's single remote. Pair again to get
  the multi-remote link.

---

## 3. Pairing, step by step

Pair **one buggy at a time**, with every other buggy switched off. Nothing can drive while the window
is open.

1. **Open the window on the buggy.** Hold **BIND** while you power the buggy on. The BIND LED blinks
   slowly while the window is open: 60 s, plus 20 s for each remote that pairs, 120 s at most.
   *Hold BIND only. BIND and AUX together at power-on delete the buggy's configuration.*
2. **Your own remote first, at arm's length.** Hold it about 1 m from the buggy, then hold **RIGHT**
   while you power it on, **with the trigger released**. The remote shows `PA` while it tries, and
   scrolls `SAVE` when it has paired. **The first remote paired is the master**, so the remote you ride
   with goes first.
   *RIGHT with the trigger squeezed at power-on deletes the remote's configuration.*
3. **Count the LED flashes.** Each time a remote pairs, the BIND LED flashes once per slot number:
   **1 flash = slot 1, the master**; 2 flashes = slot 2, a guest.
4. **Then the second remote**, the same way, at arm's length. Two flashes.
5. **Close the window.** Press BIND once, or let the time run out.
   - **LED solid for 1 s** = the new list is saved.
   - **Three fast blinks** = nobody paired in this window; the previous pairing is kept.
6. **Check with `?mtx` on the buggy** (serial, or the web console). Slot 1 must carry your own remote's
   ID. Each remote prints its own ID at boot and in its own `?mtx`.

*Example, with made-up IDs (not from any real board):*

```
buggy, ?mtx:
mode: MULTI (2-4 remotes: the ownership rules) | 2 remote(s) in the list
slot 1  5C:4A:7B  MASTER  new-style | heard 40 ms ago | ...
slot 2  5C:91:0E  guest   new-style | heard 900 ms ago | ...

remote A, boot line:
MTX: id 5C:4A:7B (eFuse), own_address 5C:4A:7B, link v3 to 5C:10:20, L 00:5A:5B
```

**If a remote does not pair** within its 5 s try, it shows `EP` again. Move it closer and press the
toggle to try again while the window is still open. If the request was too weak, the buggy's serial says
so (`refused - RSSI ... below -60`).

**Rules of the window:**

- **The list is rebuilt from each window, never added to.** Pair every remote in the same window. A
  remote you leave out is no longer paired.
- **Pairing needs a strong signal.** Both ends drop to -9 dBm while pairing and accept only frames at
  -60 dBm or stronger, which is about 5 m or closer. That figure is **calculated, not measured**: pair at
  arm's length, and check the LED count and `?mtx` afterwards. A stranger's remote within a few metres
  during the window could still pair.
- **An old-style remote** (older firmware) can pair only as the single remote, and it closes the window
  at once.

---

## 4. Who is in control: the screens

The ownership dot, the `E` codes (`E1` to `E4`, `E9`) and `tA` appear only with two or more remotes paired on the
new link; with one remote they never show. The new `St` cases at the end of the `St` list apply to every remote.

### The ownership dot

It sits in the status column right of the digits, the third dot from the top (two below the GPS dot).
It shows only while no Follow-Me dot is lit there: the Follow-Me dots win.

| Dot | Means | What you can do |
|---|---|---|
| **Solid** | You are in control. | Ride. |
| **Pulse** (two quick blinks, then a pause, every 1.2 s) | Control is available: nobody is in control, or the remote in control can be claimed (§5). | Let go of the trigger and squeeze to take control, or summon. |
| **Slow** (one short blip every 3 s) | Another remote is in control. | Wait. |
| **Off** | One remote paired, an old-style link, or no reply from the buggy for 5 s. | Nothing to act on. |

For a display check, `?mtxdot 0` to `?mtxdot 3` on the remote forces each look (0 off, 1 solid, 2 slow,
3 pulse); `?mtxdot auto` ends it.

### The codes

| Screen | Buzz | Means |
|---|---|---|
| **`E1`** to **`E4`** | two short | Another remote is in control and cannot be claimed now, so your input is ignored. **The digit is the number of the remote in control:** `E1` is the master (the remote paired first), `E2` the second remote paired, and so on (`?mtx` on the buggy lists them as slots 1-4). It shows while you squeeze or steer, and for 1.5 s after. **The other rider's motors are not affected.** It is not an error state: your next toggle press is not taken as an error acknowledgement. You also get it when you try to summon (§5c), or make a `mag_mode 4` magnet tap (§5e), while another remote is busy in control. |
| **`E9`** | two short | The same answer when your remote does not know the number of the remote in control. It is rare: you see it only after several lost answers from the buggy, or in the first seconds after your control was taken when nobody ended up in control. |
| **`tA`** | two short | Shown 3 s. Control was taken from you: another remote claimed it, or the master took it back. The buggy has already ended your Follow-Me or Return-To-Me, and your remote ends its own too. If your remote was switched off when it happened, it shows `tA` once it is heard again. |
| **`St`** | one long | It did not engage. See the list below. |

### When you see `St`

**With two remotes:**

- **Your summon was refused on your own remote** (§5c): no fresh GPS fix, that mode is switched off on
  this remote, the trigger is held, or the buggy's last answer is too old. Nothing was sent. With the
  trigger held, this `St` does not freeze the remote.
- **A magnet tap with `mag_mode 4` on a remote that is not in control**, while control is available: a
  tap never takes control (§5e). Nothing was sent.
- **Your summon got no "you are in control" answer** within 1.5 s of the summon.
- **Your take-back was not confirmed** within 1 s (§6).
- **Your remote lost control while Follow-Me or Return-To-Me was armed on it**, without a `tA` (for
  example, the buggy rebooted).

**With any number of remotes (new in this firmware):**

- **A Follow-Me or Return-To-Me gesture that cannot engage now shows `St`** instead of doing nothing:
  the mode is switched off on this remote (`fm_override_enabled`, `rtm_enabled`, `gps_en`), or a gesture
  that needs the trigger released was made with the trigger held. With the trigger held, `St` shows for
  2 s without freezing the remote.
- **A brief `St` (1 s) while your own Return-To-Me is running or arming, or your own Follow-Me is
  armed.** A Follow-Me or Return-To-Me gesture made then cannot act (or a Return-To-Me arm over Follow-Me
  cannot start). The mode continues, and its screen comes back after the second. It never freezes the
  remote.
- **The Return-To-Me arm ceremony timed out.** After `rn`, no first or no second squeeze came within the
  arm window (`rtm_arm_window_s`, default 15 s). The screen used to go blank; it now shows `St` for 2 s.

**No `St` for a blocked "end" or "change".** If the gesture would only have ended or changed a mode that
is running, and the held trigger blocks it (for example a magnet Return-To-Me hold made with the trigger
held during a return, or the Follow-Me combo with the trigger held while Follow-Me is armed), nothing
shows: the mode really continues, and its screen stays. The serial line says why. Let go of the trigger
to end a return, as today.

### A remote that is not in control

It sends once a second while nobody is using it, to keep the channel free for the rider in control. In
that slow mode it does not play the five-buzz link-lost alarm, and it counts the link as up for 2.5 s
instead of 1 s. It speeds up as soon as control is available and you squeeze or start a gesture, when
you start the master's take-back, or after 5 s with no reply.

---

## 5. Taking control

**On a remote that is not in control, only two things take control** (plus the master's take-back, §6):

1. **A squeeze from zero** (§5b), when the buggy is free or the remote in control is claimable.
2. **A summon** (§5c): your **complete** Return-To-Me or Follow-Me arm gesture, when the buggy is free or the remote
   in control is claimable. The claim happens only once the whole gesture is done - the combo's full hold, or the
   magnet's full hold, the moment that mode would arm. Never on the first tap, and never on a gesture you let go
   of early. **A magnet tap never summons either:** with `mag_mode 4` the short magnet contact (0.6-3 s) is the whole
   Follow-Me gesture, a single contact, so on a remote that is not in control it is refused and sends nothing (§5e).

**Nothing else ever takes control:** not a toggle tap, a gear change, the display cycle (throttle % or GPS speed),
the telemetry or battery views, a lock, or any setup gesture. On a remote that is not in control these send no claim
and no summon. On the remote that **is** in control, the same presses still count as "busy" (§5f), so nobody can
claim it while you use them.

### 5a. The claim rules

| The buggy is... | Another remote can take control by... |
|---|---|
| **Free:** nobody is in control (after the buggy boots, after the remote in control locks, after a change of the paired list) | a squeeze from zero (§5b) |
| **Claimable:** another unlocked remote is on, and the remote in control has been **silent for 10 s** (switched off, out of range) **or** has sat **at zero for 10 s with Follow-Me and Return-To-Me off** | a squeeze from zero, or a summon (§5c) |
| **In use:** the remote in control is driving, busy (§5f), or has Follow-Me or Return-To-Me armed and is still transmitting | nothing: it gets `E1`-`E4` (the number of the remote in control). Only the master's take-back (§6), and only with nothing armed. |

- The 10 s is `mtx_claim_s` (6-60 s). The silence is never shorter than `failsafe_time` + 1 s.
- **"Another unlocked remote is on"** means a remote other than the one in control sent a control
  packet within the last 3 s. A locked remote, or one that is switched off, does not count. **With no
  other remote on, the remote in control keeps control, however long it is silent or idle.**
- **Claimable changes nothing on the buggy.** Follow-Me or Return-To-Me stays armed, and the motors stay
  stopped (a silent remote's motors were already stopped by the failsafe; an idle one is at zero). If the
  remote in control is heard again first, or gives input, it simply carries on. Its own next squeeze is its own throttle, with no takeover limit.

### 5b. A squeeze from zero

Let go of the trigger fully, then squeeze. The buggy must see your trigger at zero first (two packets in
a row, the last within 3 s), and then the squeeze. **A trigger that was already held never takes
control.**

If Follow-Me was running on the buggy, the buggy ends it first and holds the claim for you, and only
you, for up to 1 s while it stops. If that runs out, let go and squeeze again.

### 5c. A summon

A summon takes control **and** brings the buggy to you. Make your normal **Return-To-Me gesture** (RIGHT
tap, then LEFT hold, or your magnet's Return-To-Me hold) or your normal **Follow-Me gesture** (LEFT tap,
then RIGHT hold, or your magnet's Follow-Me hold; never the `mag_mode 4` tap, §5e), **with the trigger
released**, while your ownership dot pulses.

- **Your remote checks first.** The buggy's latest answer must say control is available and be under
  5 s old, your remote must have a **fresh GPS fix** (the same test that lets it send its GPS to the
  buggy), that mode must be switched on, and the remote must not be locked. If any check fails, **nothing
  is sent**: you see `E1`-`E4` (the number of the remote in control) if another remote is busy in control,
  `St` otherwise.
- **The buggy checks too.** It accepts only if nobody is in control or the remote in control is
  claimable, and your trigger was at zero within the last second. **A summon can never take control from
  a remote that is driving, or that has Follow-Me or Return-To-Me armed and is still transmitting.**
- **Then today's normal arm runs.** Once the buggy names your remote the one in control (its first "you
  are in control" answer within 1.5 s of the summon, however long you keep the button held): for
  Return-To-Me the `rn` screen and its squeezes, for Follow-Me the `F<n>` label. Every normal check
  applies: your GPS, the distance cross-check, the buggy's own GPS and compass. **Nothing moves until you
  squeeze.**
- **With Return-To-Me, control passes to you when `rn` appears**, at the end of your combo or magnet hold,
  before the two squeezes: the buggy accepts the arm only from the remote in control. If you then let the
  `rn` window run out, nothing is armed, you keep manual control, and the other rider has lost it.
- **Nothing is armed after all** if, by the time the arm would start, you are squeezing the trigger, have
  armed that mode yourself, or are no longer in control. You keep manual control.
- **If the buggy refuses the arm** (for example, you are too far away), you keep manual control and see
  today's `St` or not-ready screen.
- **A late answer.** If the buggy gave you control after your remote's 1.5 s wait, your remote shows `St`
  and arms nothing, but you are in control: the dot goes solid. Make the gesture again.

### 5d. The takeover soft start

Every time control passes to a remote (a squeeze on a free buggy, a claim, a summon, the master's
take-back), that remote's throttle is **limited to 25 % at that moment, rising evenly to full over 4 s**
(`mtx_takeover_cap` 64 of 255, `mtx_takeover_ms` 4000). The buggy may be close to the other rider when
control passes.

| Time after control passes | 0 s | 1 s | 2 s | 3 s | 4 s |
|---|---|---|---|---|---|
| Throttle limit (of 255) | 64 | 111 | 159 | 207 | 255 = full |

It only lowers throttle, never reads GPS, and ends on its own. The motors' normal soft start still runs
as well. A remote that never lost control (for example, one coming back after a silence) gets no limit.
With one remote paired it never applies, except once: right after the paired list changes from two or
more remotes to one (§10a).

### 5e. What never takes control

- **Plain button presses never take control.** On a remote that is not in control, a toggle tap changes
  nothing: no gear, no cap. The display cycle and the telemetry views only change your own screen. Only a
  squeeze from zero or a **completed** summon gesture can take control (§5).
- **A gesture you let go of early never takes control**, and neither does its first tap.
- **A magnet tap never takes control.** With `mag_mode 4` the tap (0.6-3 s) is a single contact and the whole
  Follow-Me gesture, so on a remote that is not in control it sends nothing and shows `E1`-`E4` (another remote is
  busy in control) or `St`. Take control there with the 5 s magnet hold (Return-To-Me), the Follow-Me toggle combo,
  or a squeeze from zero. On the remote in control the tap works as before.
- **A held trigger never takes control**, also after a buggy reboot.

### 5f. The remote in control stays busy

The remote in control counts as **busy**, never idle, while you:

- hold any throttle (from about 3 %) or steer;
- press or hold the toggle, hold the magnet, or run the Return-To-Me arm ceremony. While a press is in
  progress, and for 3 s after the last one, the remote tells the buggy "busy" at most every 2 s.

So a rider who stops and works through a gesture is never claimed halfway. Sitting at zero with nothing
pressed and nothing armed for 10 s is idle.

---

## 6. The master's take-back

The master (slot 1) can take control back from a guest, also while the guest is driving.

**The gesture:** trigger released, **RIGHT tap**, then press **RIGHT** again within 0.4 s and **hold it
1.2 s** (`mtx_seize_hold_ms`, 1000-1800 ms).

- **It works only while the buggy offers it:** a guest is in control, `mtx_seize_en` is 1, and Follow-Me
  and Return-To-Me are off. **While the guest has Follow-Me or Return-To-Me armed it is refused**, and the
  gesture does nothing.
- Any throttle, an early release, or a change in the buggy's answer aborts it. The held second press does
  not cycle the display.
- **It stops the guest's buggy at once.** The guest's throttle goes to zero the moment the buggy accepts
  the take-back, about 1.6 s after your first press, and the guest's remote shows `tA`. **Use it with
  care: do not take back while the guest depends on the buggy** (on the rope, or far out). It can only
  bring the buggy to a stop; it can never start it.
- **The answer on your remote** comes while you still hold RIGHT: two quick taps (the arm-confirm buzz)
  = you are in control, under the takeover soft start. `St` = the buggy did not hand control back within
  1 s of the take-back. Holding RIGHT longer does not change the answer.
- To switch it off, set `mtx_seize_en 0` on the buggy.

---

## 7. Follow-Me and Return-To-Me with two remotes

- **Only the remote in control** can arm Follow-Me or Return-To-Me, change the Follow-Me station, and use
  aux. Only its GPS is used: the buggy follows or returns to **that** rider.
- **A remote that is not in control** turns its Return-To-Me or Follow-Me gesture into a **summon** (§5c).
  Its screen never shows the other rider's Follow-Me or Return-To-Me. Battery, temperature, speed, the
  water alarm and its own link quality still show; while another remote is in control they update at about
  half the usual rate, because every other answer from the buggy carries the number of the remote in control.
  **While you squeeze it** (and see `E1`-`E4`), every answer carries that number, so they do not update at all
  until you let go.
- **While Follow-Me or Return-To-Me is armed, the remote in control cannot be claimed by sitting idle**,
  however long. Only 10 s of silence makes it claimable, and the master cannot take back.
- **When control passes, the buggy ends Follow-Me and Return-To-Me first** and drops the previous rider's
  GPS position, so it can never steer toward where that rider was. The new rider arms his own (a summon
  does that for him). The previous rider's remote shows `tA` and drops its Follow-Me screen.
- **Losing your GPS fix during Follow-Me** works as today: Follow-Me pauses (not ready) and ends after 30 s
  with `St`. It never makes your remote claimable, because it is still transmitting.

**Two typical cases:**

- **The GPS rider is in Follow-Me and his remote goes silent for 10 s.** The other rider can take over only
  by squeezing from zero. That ends Follow-Me, and he drives manually.
- **The rider without GPS is in control, and the GPS master wants Follow-Me.** The master takes back (this
  stops the buggy at once), then arms Follow-Me normally. Or, once the other remote has been at zero (or
  silent) for 10 s, the master summons with his Follow-Me gesture.

---

## 8. When a remote goes silent

- **The motors stop by the failsafe**, `failsafe_time` (default 1 s) after the last control packet from the
  remote in control. Another remote's packets can never keep them running.
- **Nothing ends on a timer.** No multi-remote timer releases control, and none ends Follow-Me or
  Return-To-Me. They keep their own link-loss rules exactly as with one remote: for example, Follow-Me
  pauses and ends after 30 s without the link ([Follow-Me guide §6](FOLLOW_ME_GUIDE.md)). A remote that
  goes to sleep switches its own Follow-Me and Return-To-Me off first, as today.
- **After 10 s of silence**, and only with another unlocked remote on, the silent remote becomes claimable
  (§5a). Until someone claims, nothing changes on the buggy. If the silent remote comes back first, it is
  still in control, and the motors start again from zero through the normal soft start.
- **If the buggy reboots** with two remotes paired, nobody is in control. Let go of the trigger and squeeze
  to take control (under the takeover soft start). A held trigger never restarts the buggy.
- **With one remote paired, everything is as today.** The same holds with the second remote switched off
  or locked, with one difference: after your own lock or a buggy reboot nobody is in control, so your next
  squeeze from zero takes control under the takeover soft start.

---

## 9. The spare remote locks itself

- A remote that is **not in control** on a multi-remote buggy **locks itself after 120 s untouched**
  (`mtx_autolock_s`; 0 = off, otherwise 30-600 s). Untouched means no throttle (above about 8 %) and no
  steering. Toggle presses at zero throttle do not count.
- **A locked remote cannot take control or summon**, and it does not count as "another remote on". So a
  spare in a bag never makes the rider's remote claimable.
- The auto-lock waits while Follow-Me or Return-To-Me is armed on that remote, during the take-back
  gesture, and in a menu or in setup.
- Unlock it as usual: LEFT hold, then squeeze the trigger, then let go. The unlock does not finish while
  the trigger is still pulled.
- A remote that boots locked (`no_lock 0`, the default) and is never unlocked does not count as "on"
  either.
- `autolock_timeout_s` is a separate auto-lock for a single remote, or for the remote in control: default
  0 = off (today's behaviour), 5-180 s when set.

> **Warning: `no_lock 1` switches all of this off.** With `no_lock 1` a remote boots unlocked, the LEFT
> hold does not lock it, and the spare auto-lock does not run. The remote prints
> `NOTE: no_lock is set - a spare remote will not lock itself` at boot.
> **With `no_lock 1`, switch off every remote you are not holding.**

---

## 10. Changing the paired list, and loading settings files

### 10a. A list change while the buggy is running

Apart from the pairing window at power-on, the paired list changes only when you load a configuration
(`?setconf` then `?applyconf`, or the web import) or set `?set paired 0`. If that happens while the buggy
runs, and two or more remotes were paired before or after the change:

- **the throttle goes to zero and the steering to centre at once**, and the remote in control is released
  (Follow-Me and Return-To-Me off);
- afterwards, with two or more remotes paired, nobody is in control: the next squeeze from zero takes
  control under the takeover soft start. With one remote left, that remote is obeyed at once, with no
  squeeze from zero needed, and its throttle goes through the takeover soft start once (25 % rising to
  full over 4 s, §5d) as well as the motors' normal soft start. A trigger that was already squeezed goes
  straight to at most 25 %, rising, without passing through zero. After that it is a single remote, as
  today.

With one remote before and after the change, this is today's behaviour. Make such changes on the bench,
with the trigger released.

### 10b. Loading another board's settings file

| You load... | On the buggy |
|---|---|
| **A JSON settings file made on another buggy** | Its settings are imported, its pairing is not: **this buggy keeps its own pairing**, and one NOTE line says so. |
| A JSON file made on this buggy | Imported as before, pairing included. |
| A Base64 backup (`?conf`, `?setconf`) made on this buggy | Restores everything, the remote list included. |
| A Base64 backup made on another buggy | The settings come across, but the pairing is dropped (a NOTE line says so). Pair again. |

**On a remote**, a JSON file imports as it always did. Only the link type (`mtx_link_proto`) is never
imported, because pairing sets it. After loading another remote's file, check `?mtx`, and pair again if
the remote no longer connects.

---

## 11. Settings

### Buggy (RX): web group "Multi-Remote"

All four act only with two or more remotes paired, with one exception: right after the paired list changes from
two or more remotes to one, the takeover limit (`mtx_takeover_cap`, `mtx_takeover_ms`) applies once (§10a).

| Setting (web label) | Default | Range | What it does |
|---|---|---|---|
| `mtx_claim_s` (Claim time with two remotes) | **10 s** | 6-60 s | With another remote on: how long the remote in control must be silent, or sit at zero with Follow-Me and Return-To-Me off, before another remote may claim control. It releases nothing by itself. The silence is never shorter than `failsafe_time` + 1 s. |
| `mtx_seize_en` (Master take-back) | **1** | 0-1 | 1 = the master may take control back from a guest (§6). 0 = no take-back. |
| `mtx_takeover_cap` (Takeover throttle limit) | **64** (25 %) | 0-255 | Throttle limit at the moment control passes (§5d). 255 = no limit. |
| `mtx_takeover_ms` (Takeover limit time) | **4000 ms** | 0-10000 ms | Time for that limit to rise to full. 0 = no limit. Never more than 10 s. |

### Remote (TX): web group "Multi-Remote"

| Setting (web label) | Default | Range | What it does |
|---|---|---|---|
| `mtx_autolock_s` (Spare Auto-Lock) | **120 s** | 0, or 30-600 s | A remote that is not in control locks itself after this long untouched (§9). 0 = off. 1-29 are raised to 30. |
| `autolock_timeout_s` (Auto-Lock Timeout) | **0** (off) | 0, or 5-180 s | Auto-lock for a single remote, or the remote in control. 1-4 are raised to 5. |
| `mtx_seize_hold_ms` (Master Take-Back Hold) | **1200 ms** | 1000-1800 ms | How long the master holds RIGHT for the take-back (§6). |
| `mtx_link_proto` (Link Type (set by pairing)) | 0 | 0 or 3, read-only | 0 = old-style link (a pairing carried over from older firmware), 3 = new-style multi-remote link. Pairing writes it. |

### Existing settings that matter here

| Setting | Default | Note |
|---|---|---|
| `failsafe_time` *(RX)* | 1000 ms | The motors stop this long after the last control packet. The claim silence is never shorter than this + 1 s. |
| `no_lock` *(TX)* | 0 | 1 = no lock, and no spare auto-lock (§9). |
| `own_address` *(RX)* | - | Read-only now: it is the buggy's chip ID, worked out at every boot. |

**Commands:** `?mtx` on both boards (the list, who is in control, claimable, counters); `?mtxdot` on the
remote (§4).

---

## 12. Safety rules before you test

1. **Bench first.** Props off, drivetrain secured.
2. **One buggy at a time.** Switch every other receiver off, for pairing and for every multi-remote test.
   Never power two buggies during a multi-remote test.
3. **Check for cloned buggies.** Today's firmware obeys any remote addressed to the buggy's stored
   `own_address`. If a configuration was ever copied between receivers, two buggies can answer one remote.
   Check with `?conf` that no two receivers show the same `own_address`; this matters most for a buggy
   still on the old firmware. (On this firmware the address is the chip ID, and a copied configuration
   drops its pairing at boot.)
4. **Pair your own remote first, at arm's length**, then check the LED count and `?mtx` (§3). If your
   remote ended up as a guest, it cannot stop a spare that stays squeezed; that is why your remote goes
   first.
5. **The pairing distance is calculated, not measured.** The -60 dBm gate and "about 5 m" come from a link
   budget, not from a hardware test yet.
6. **With `no_lock 1`, switch off every remote you are not holding** (§9).
7. **Use the take-back with care** (§6): it stops the guest's buggy at once.
8. **Back up every board first** (§2).

---

## 13. Bench checklist (short)

Props off, one buggy, every other receiver off. The full plan, step by step, is in
[`MULTI_TX_DESIGN.md` §14](MULTI_TX_DESIGN.md) (host tests, builds, bench phases A to C).

- [ ] Backups of every board saved; old logs removed from the buggy.
- [ ] First boot: read the buggy's `MTX:` line and any NOTE about a dropped pairing.
- [ ] One remote paired: everything behaves as today, and no `MTX:` lines appear.
- [ ] Pair both remotes in one window: 1 flash, then 2 flashes, then solid 1 s; `?mtx` shows MULTI with
      your remote in slot 1.
- [ ] Dots: solid on the remote in control, slow on the other. A squeeze on the other gives `E1` while the
      master is in control (`E2` on the master while the second remote is), and the motors do not react.
- [ ] Idle claim: the remote in control at zero for 10 s, nothing armed: the other dot pulses. A squeeze
      from zero on it takes control under the 25 % to full soft start; the first remote shows `tA`.
- [ ] Silence: switch the remote in control off mid-throttle. The motors stop within about 1 s, and the
      other remote cannot restart them. It becomes claimable at 10 s and is never released on its own.
- [ ] Master take-back with the guest at low throttle: the motors go to zero, the guest shows `tA`.
      Release RIGHT at once, and once after holding it 3 s: the confirm buzz every time, never `St`.
- [ ] Follow-Me armed and the remote in control switched off: claimable but still armed; switched on again
      first, nothing has changed.
- [ ] Summon by each gesture (needs the buggy's GPS and a fix on the summoning remote). Refused against a
      remote that is driving (`E1`-`E4`, its number), and from a remote with no fix (`St`, nothing sent). A
      toggle summon held 2 s past the combo still arms (no `St`).
- [ ] With the other remote claimable: a toggle tap, a gear change, the display cycle and a combo let go
      before its hold time take nothing (`?mtx` on the buggy: no claim); only the completed gesture does.
- [ ] With `mag_mode 4` and the other remote claimable: one accidental magnet contact (0.6-3 s) on the spare
      takes nothing (`St`, `?mtx` on the buggy: no summon); its 5 s magnet hold does.
- [ ] During a return, a magnet Return-To-Me hold with the trigger held: no `St`, the return continues.
- [ ] Spare auto-lock after 120 s untouched.
- [ ] Pairing distance: a new-style remote refused far away, accepted at about 1 m (read the RSSI lines on
      both boards).

---

## 14. Going back to the old firmware

1. Flash the firmware you ran before on the buggy **and** on every remote (for this fork: receiver SW36,
   remote SW27).
2. Its first boot finds a newer configuration and resets the board to defaults, unpaired.
3. Restore each board's own Base64 backup from §2 (`?setconf <backup>`, or the web tool's Base64 import),
   then `?reboot`.

Each board is back where it was, pairing and calibration included. Go back with all boards together: an
older buggy does not answer a new-style pairing (the remote shows `EP`), and an older remote works with a
new buggy only as its single remote.

---

## 15. Not supported yet

- **Several buggies at once.** The packets already keep buggies apart (a frame built for one buggy is
  ignored by another), but this case is not supported or tested yet. Run one buggy at a time.
- **More than two remotes.** The buggy lists up to four; only two are in the test plan.
- **US FCC.** This feature does not change the radio settings, and it neither fixes nor worsens whether
  the fixed 250 kHz channel fits the US FCC rules. That question is out of scope here.

---

## Safety summary (never changes)

1. The buggy moves only while the rider in control holds the throttle trigger.
2. One remote in control at a time. Another remote's packets never keep the motors running.
3. No timer hands control over. It passes only by a lock, a deliberate claim, or the master's take-back.
4. Control passes only from the new remote's zero throttle, and then under the takeover soft start.
   After a list change from two or more remotes to one, the remaining remote is obeyed at once: a held
   trigger goes straight to at most 25 %, rising to full over 4 s, without passing through zero (§10a).
5. A held trigger never takes control.
6. With one remote paired, the buggy runs today's code.

*See [`MULTI_TX_DESIGN.md`](MULTI_TX_DESIGN.md) for the full engineering design and test plan,
[`FOLLOW_ME_GUIDE.md`](FOLLOW_ME_GUIDE.md) for Follow-Me itself, and `BUGGY_FOIL_DOMAIN.md` for the
safety model.*
