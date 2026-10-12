#!/usr/bin/env python3
# ============================================================
# gen_fm_bar_gifs.py - draws the Follow-Me screen GIFs for the docs
# ============================================================
# V2.5-Evo - 2026-10-12 - new: the waiting bar breathes, not-ready shows only the two end dots, following is solid.
# V2.5-Evo - 2026-10-12 - fm-returning.gif added, drawn from the firmware: the return bar BLINKS 1000 ms on / 500 ms off
#   (it used to be drawn solid), shrinks on the firmware's square-root curve, and the metres under 10 m show "X.X".
#
# What it does: renders the remote's 10 x 7 LED screen for four Follow-Me states and writes them as looping GIFs:
#   fm-waiting.gif   armed and ready: the centre distance bar BREATHES (400 ms on, 1200 ms off), one dot hopping
#   fm-notready.gif  armed, not ready or paused: only the two end dots C0 + C9 blink (1000 ms on, 500 ms off)
#   fm-following.gif following: the same centre distance bar, SOLID, two steady dots, metres counting down
#   fm-returning.gif coming to you (auto-return or return-to-me): the left-anchored return bar BLINKING 1000/500 and
#                    shrinking to the left, the dot column filling down, metres counting down (tenths under 10 m)
#
# The rules and timings are copied from the TX firmware, Source/V2_Integration_Tx/Display.ino:
#   fmCentreBarMask()        the centre bar: half_width = 1 + round(4 x distance / fm_warn_distance_m), 1..5
#   updateR5ProximityBar()   kFmBreathOnMs 400, kFmBreathCycleMs 1600; the 1000/500 blink for not ready
#   drawReturnBar()          the return bar: round(sqrt(distance / start distance) x 10) dots from C0, drawn only in
#                            the "on" part of the same 1000/500 blink (r5_blink_state)
#   updateBargraphs()        Follow-Me dots at C7: waiting = R3 and R4 take turns every 200 ms, following = both,
#                            returning = R2, R2+R3, R2+R3+R4, dark (200 ms steps)
#   displayDistanceInUnits() "X.X" with the C3 R3 dot under 10 m, whole metres from 10 to 99 m
# If those change, change the constants below and run this script again.
#
# The drawing style (LED size, spacing, colours, the 3 x 5 digits, the GPS / temperature / signal / battery
# chrome) matches docs/followme-screens.html and the older GIFs in docs/img/fm/, so all the screen GIFs look alike.
#
# Usage:  python Tools/gen_fm_bar_gifs.py                 (writes into docs/img/fm/ next to this Tools folder)
#         python Tools/gen_fm_bar_gifs.py --out some/dir --warn-m 150 --frames
# Inputs: --out folder, --warn-m the remote's fm_warn_distance_m (factory default 150), --frames also saves one PNG
#         per GIF frame so the look can be checked by eye.
# Outputs: the four GIFs (and PNG frames with --frames). Side effects: creates the output folder if needed.
# Needs: Pillow (pip install pillow).
# ============================================================

import argparse
import math
import os

from PIL import Image, ImageDraw

# ---- Firmware constants (TX Display.ino) ----
BREATH_ON_MS = 400          # kFmBreathOnMs: the waiting bar is lit for the first 400 ms of each breath
BREATH_CYCLE_MS = 1600      # kFmBreathCycleMs: 400 on + 1200 off
BLINK_ON_MS = 1000          # r5_blink_state: on 1000 ms (not-ready end dots, return bar)
BLINK_OFF_MS = 500          # r5_blink_state: off 500 ms
DOT_FRAME_MS = 200          # Follow-Me dots: one frame every 200 ms
FM_WARN_M_DEFAULT = 150     # TX defaultConf.fm_warn_distance_m (the bar's full scale), range 50-164
# The return demo (same distances as the screens page): the buggy starts 40 m away - the bar's 100 % reference, which the
# firmware captures from the first distance it shows (rtm_arm_dist_m / fm_return_ref_m) - and closes to 3 m. The loop is
# 12 s: a whole number of 1.5 s blinks and 0.8 s dot cycles, so the GIF loops without a hiccup.
RETURN_START_M = 40.0
RETURN_END_M = 3.0
RETURN_LOOP_MS = 12000

# ---- Drawing style (same as docs/followme-screens.html and the older docs/img/fm GIFs) ----
COLS, ROWS = 10, 7
PITCH = 26                  # pixels between LED centres
FIRST = 27                  # centre of C0 / R0
R_LED = 9.25                # lit core and dark LED radius
R_GLOW = 12.5               # glow ring around a lit LED
WIDTH = FIRST * 2 + PITCH * (COLS - 1)    # 288
HEIGHT = FIRST * 2 + PITCH * (ROWS - 1)   # 210
CASE = (34, 40, 42)
LED_OFF = (58, 26, 23)
GLOW = (120, 40, 32)
LED_ON = (255, 74, 54)
PALETTE = [CASE, LED_OFF, GLOW, LED_ON]

# 3 x 5 digit glyphs, as in the screens page
GLYPHS = {
    "0": ["111", "101", "101", "101", "111"], "1": ["010", "110", "010", "010", "111"],
    "2": ["111", "001", "111", "100", "111"], "3": ["111", "001", "111", "001", "111"],
    "4": ["101", "101", "111", "001", "001"], "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"], "7": ["111", "001", "001", "001", "001"],
    "8": ["111", "101", "111", "101", "111"], "9": ["111", "101", "111", "001", "111"],
    "-": ["000", "000", "111", "000", "000"], " ": ["000", "000", "000", "000", "000"],
}


def round_half_up(x):
    """Rounds like the firmware's (uint8_t)(x + 0.5f) and JavaScript Math.round for x >= 0."""
    return int(math.floor(x + 0.5))


def blank():
    """An all-dark 7-row x 10-column screen (grid[row][col])."""
    return [[False] * COLS for _ in range(ROWS)]


def chrome(g):
    """The parts that do not change in these demos: GPS dot solid (C7 R0), a short temperature bar (C8 R3-R4),
    a good signal bar (C9 R1-R4) and the battery bar (R6, C0-C6)."""
    g[0][7] = True
    for r in (3, 4):
        g[r][8] = True
    for r in range(1, 5):
        g[r][9] = True
    for c in range(7):
        g[6][c] = True


def digits(g, text):
    """Two characters in the digit zone, C0-C2 and C4-C6, rows R0-R4 (right-aligned like the screens page)."""
    s = str(text).rjust(2)[-2:]
    for i, c0 in enumerate((0, 4)):
        glyph = GLYPHS.get(s[i], GLYPHS[" "])
        for r in range(5):
            for c in range(3):
                if glyph[r][c] == "1":
                    g[r][c0 + c] = True


def telemetry_m(d):
    """The distance as the buggy sends it: tenths of a metre under 10 m, whole metres from 10 m."""
    return round_half_up(d * 10) / 10.0 if d < 10 else float(round_half_up(d))


def distance_digits(g, d):
    """displayDistanceInUnits(): under 10 m "X.X" with the true decimal dot at C3 R3, else whole metres (10-99)."""
    if d < 10:
        tenths = min(99, round_half_up(d * 10))
        digits(g, "%d%d" % (tenths // 10, tenths % 10))
        g[3][3] = True
    else:
        digits(g, str(min(99, round_half_up(d))))


def fm_centre_bar(g, d, warn_m):
    """fmCentreBarMask(): centre-expanding from C4+C5, grows with distance, full at fm_warn_distance_m, never empty."""
    ref = float(warn_m) if warn_m >= 1 else 30.0
    ratio = min(1.0, d / ref)
    half = min(5, 1 + round_half_up(ratio * 4.0))
    for i in range(half):
        g[5][4 - i] = True
        g[5][5 + i] = True


def dots_waiting(g, t):
    """W1: R3 and R4 at C7 take turns every 200 ms, R3 first - exactly one lit."""
    g[3 if (t // DOT_FRAME_MS) % 2 == 0 else 4][7] = True


def dots_following(g):
    """F1: R3 + R4 at C7 steady."""
    g[3][7] = True
    g[4][7] = True


def dots_returning(g, t):
    """R1: the C7 column fills down from the top - R2, R2+R3, R2+R3+R4, then dark - one step every 200 ms."""
    f = (t // DOT_FRAME_MS) % 4
    if f < 3:
        g[2][7] = True
        if f >= 1:
            g[3][7] = True
        if f >= 2:
            g[4][7] = True


def return_bar(g, d, ref_m, t):
    """drawReturnBar(): left-anchored from C0, round(sqrt(d / ref) x 10) dots (0..10), full at the start distance and
    shrinking toward the left as the buggy closes - drawn only in the "on" part of the 1000/500 blink."""
    if (t % (BLINK_ON_MS + BLINK_OFF_MS)) >= BLINK_ON_MS:
        return                                   # off phase: R5 dark
    if ref_m <= 0:
        return
    ratio = min(1.0, d / ref_m)
    n = min(10, round_half_up(math.sqrt(ratio) * 10.0))
    for c in range(n):
        g[5][c] = True


# ---- The three states ----

def frame_waiting(t, d_fn, warn_m):
    """Armed and ready: the centre bar breathes - width latched at the start of each 1600 ms breath, lit 400 ms."""
    g = blank()
    chrome(g)
    d = telemetry_m(d_fn(t))
    distance_digits(g, d)
    breath_start = t - (t % BREATH_CYCLE_MS)
    if (t - breath_start) < BREATH_ON_MS:
        fm_centre_bar(g, telemetry_m(d_fn(breath_start)), warn_m)
    dots_waiting(g, t)
    return g


def frame_notready(t):
    """Armed, not ready or paused: only C0 and C9 on R5, blinking 1000 on / 500 off; the distance shows "--"."""
    g = blank()
    chrome(g)
    digits(g, "--")
    if (t % (BLINK_ON_MS + BLINK_OFF_MS)) < BLINK_ON_MS:
        g[5][0] = True
        g[5][9] = True
    dots_waiting(g, t)
    return g


def frame_following(t, d_fn, warn_m):
    """Following: the same centre bar, solid, from the live distance."""
    g = blank()
    chrome(g)
    d = telemetry_m(d_fn(t))
    distance_digits(g, d)
    fm_centre_bar(g, d, warn_m)
    dots_following(g)
    return g


def frame_returning(t, d_fn, ref_m):
    """Coming to you (auto-return or manual return-to-me, same look): the blinking return bar, the dot column filling
    down, and the live distance in the digits (tenths with the dot under 10 m)."""
    g = blank()
    chrome(g)
    d = telemetry_m(d_fn(t))
    distance_digits(g, d)
    return_bar(g, d, ref_m, t)
    dots_returning(g, t)
    return g


def follow_demo_m(t):
    """The screens page's following demo: closes from 15 m to 8 m over 2.8 s, holds 8 m, loops every 3.6 s."""
    k = t % 3600
    return 15 - 7 * (k / 2800.0) if k < 2800 else 8.0


def return_demo_m(t):
    """The return demo: closes steadily from RETURN_START_M to RETURN_END_M over one RETURN_LOOP_MS loop."""
    k = t % RETURN_LOOP_MS
    return RETURN_START_M - (RETURN_START_M - RETURN_END_M) * (k / float(RETURN_LOOP_MS))


# ---- Rendering ----

def render(g):
    """Draws one screen as a palette image in the screens-page style."""
    im = Image.new("P", (WIDTH, HEIGHT), 0)
    im.putpalette([v for rgb in PALETTE for v in rgb])
    px = im.load()
    for r in range(ROWS):
        cy = FIRST + PITCH * r
        for c in range(COLS):
            cx = FIRST + PITCH * c
            lit = g[r][c]
            reach = int(math.ceil(R_GLOW))
            for dy in range(-reach, reach + 1):
                for dx in range(-reach, reach + 1):
                    dd = dx * dx + dy * dy
                    if dd <= R_LED * R_LED:
                        px[cx + dx, cy + dy] = 3 if lit else 1
                    elif lit and dd <= R_GLOW * R_GLOW:
                        px[cx + dx, cy + dy] = 2
    return im


def frames_over(period_ms, step_ms, make):
    """Samples make(t) every step_ms over one loop and merges identical neighbours into longer frames."""
    out = []   # [grid, duration_ms]
    for t in range(0, period_ms, step_ms):
        g = make(t)
        if out and out[-1][0] == g:
            out[-1][1] += step_ms
        else:
            out.append([g, step_ms])
    if len(out) > 1 and out[0][0] == out[-1][0]:   # the loop seam: fold the last frame into the first
        out[0][1] += out.pop()[1]
    return out


def write_gif(path, frames, png_dir=None):
    """Writes a looping GIF; frame durations are the firmware times in milliseconds."""
    images = [render(g) for g, _ in frames]
    durations = [ms for _, ms in frames]
    images[0].save(path, save_all=True, append_images=images[1:], duration=durations, loop=0,
                   optimize=False, disposal=1)
    if png_dir:
        base = os.path.splitext(os.path.basename(path))[0]
        for i, (im, ms) in enumerate(zip(images, durations)):
            im.convert("RGB").save(os.path.join(png_dir, "%s_%02d_%dms.png" % (base, i, ms)))
    return durations


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Draw the Follow-Me screen GIFs from the TX firmware rules.")
    ap.add_argument("--out", default=os.path.join(here, "..", "docs", "img", "fm"), help="output folder")
    ap.add_argument("--warn-m", type=int, default=FM_WARN_M_DEFAULT,
                    help="fm_warn_distance_m, the bar's full scale in metres (factory default 150)")
    ap.add_argument("--frames", action="store_true", help="also save every frame as a PNG for checking")
    a = ap.parse_args()

    out = os.path.normpath(a.out)
    os.makedirs(out, exist_ok=True)
    png_dir = None
    if a.frames:
        png_dir = os.path.join(out, "_frames")
        os.makedirs(png_dir, exist_ok=True)

    waiting_m = lambda t: 15.0   # the rider on the rope, 15 m from the buggy, as on the screens page
    jobs = [
        # name, loop length (ms), sample step (ms), frame maker. 1.6 s = one breath and four dot cycles.
        ("fm-waiting.gif", BREATH_CYCLE_MS, 100, lambda t: frame_waiting(t, waiting_m, a.warn_m)),
        ("fm-notready.gif", 6000, 100, frame_notready),   # 6 s = the 1.5 s blink and the 0.4 s dot cycle both repeat
        ("fm-following.gif", 3600, 200, lambda t: frame_following(t, follow_demo_m, a.warn_m)),
        # 100 ms steps: the blink's 500 ms off phase is not a multiple of 200 ms.
        ("fm-returning.gif", RETURN_LOOP_MS, 100, lambda t: frame_returning(t, return_demo_m, RETURN_START_M)),
    ]
    for name, period, step, make in jobs:
        frames = frames_over(period, step, make)
        durs = write_gif(os.path.join(out, name), frames, png_dir)
        print("%-18s %2d frames, %5d ms loop: %s" % (name, len(durs), sum(durs), durs))


if __name__ == "__main__":
    main()
