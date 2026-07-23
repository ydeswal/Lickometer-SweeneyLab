# LIQ HD v2.1 — lick resolution inside bouts, and a Sweeney Lab interface

Two changes: the reason individual licks were going missing during continuous
drinking, and a rebuilt interface.

---

# Part 1 — why licks disappeared in the middle of a bout

## The mechanism

Your description was the diagnostic clue: **bouts accurate, individual licks not**.
That pattern only has one real explanation.

When a mouse settles in and drinks continuously, the reading does not return to the
idle baseline between tongue contacts. The snout sits at the spout, liquid bridges
the gap, and the animal's body adds a steady DC offset for as long as it stays there.
Individual licks then ride on top of that offset instead of returning to zero.

Your release threshold is 2 counts. If the between-lick floor sits anywhere above 2 —
and with an animal parked at the sipper it easily does — the detector **never
releases**. A whole run of licks is recorded as one continuous touch, ended only by
the 1500 ms stuck-channel watchdog.

Bout statistics survive this untouched, because the bout still begins at the first
contact and ends at the last. Only the lick count inside it collapses. That is
exactly the asymmetry you observed.

## How bad it was

I ported the detector to Python and ran a simulated bout: 40 licks at 7 Hz, 50 ms
contacts, 40-count amplitude, with a 12-count DC offset, using your own thresholds
(touch 5, release 2).

| detector configuration | licks found (of 40) |
|---|---|
| **your current firmware** | **5.0  (12%)** |
| relative release only | 41.0 |
| plateau tracker only | 5.8 |
| **both — new default** | **40.0  (100%)** |

Five out of forty. The watchdog was firing roughly once every 1.5 s and each forced
release counted as a single lick, which is why you still got *something* rather than
nothing.

## The fix — two mechanisms, both switchable

**1. Plateau tracker (AC coupling).** Estimates the sustained DC component and
subtracts it, so each lick is measured from the current local floor rather than the
idle level. It rises slowly (~500 ms) so a 30–70 ms lick is never tracked out, and
falls quickly (~32 ms) so the floor between contacts is re-acquired well inside an
inter-lick gap. It is frozen while a contact is confirmed, so it cannot chase the
lick and corrupt the measured duration.

**2. Relative release.** Releases when the signal has fallen to a set fraction of
*that contact's own peak*, not only when it reaches an absolute floor. A tongue
withdrawal is a large fractional drop even when it sits on a DC pedestal.

Note from the table that neither alone is sufficient, and for different reasons. The
plateau tracker is frozen during a touch, so on its own it cannot break a merge that
has already started. Relative release on its own breaks the merge but slightly
over-counts (41 of 40), because after each release the signal is still above
threshold and re-triggers a fraction early. Together they land exactly on 40.

## Robustness

Same detector, other conditions:

| scenario | found | expected |
|---|---|---|
| clean licking, no DC offset | 40.0 | 40 |
| heavy DC offset (30 counts) | 40.4 | 40 |
| fast licking, 9 Hz | 40.0 | 40 |
| weak licks (amplitude 15) | 40.0 | 40 |
| slow drinker, 4 Hz | 40.2 | 40 |
| **idle rig, noise only** | **0.0** | **0** |

The phantom-lick behaviour you currently have is preserved — nothing in this change
lowers the noise floor. `release_drop_percent` was also swept from 25 to 70 and gave
40.0 licks at every value, so it is not a knob you will have to fight.

## Second cause: a blind window at every time bin

Smaller, but real. The old sequence at each bin was: stop sampling → run
`SD.begin()` → write the row → zero the counters. Nothing was sampled during that
window, and any lick that did land between the write and the reset was zeroed before
it reached the card. Every minute, plus a real FAT flush every `SYNC_INTERVAL`.

Now the counters are **snapshotted**, the snapshot is logged while sampling
continues, and the snapshot is **subtracted** rather than the counters being zeroed.
Licks that land mid-write carry into the next bin instead of vanishing. Sampling
calls are interleaved between the logged fields and around the SD health check.

## New settings

```cpp
bool use_plateau_tracker = true;
int  plateau_rise_shift = 7;      // how slowly the DC estimate rises  (7 = ~500 ms)
int  plateau_fall_shift = 3;      // how quickly it falls to the floor (3 = ~32 ms)
int  plateau_max = 400;           // sanity clamp, in counts

bool use_relative_release = true;
int  release_drop_percent = 40;      // release at this % of the contact's peak
int  relative_release_min_peak = 2;  // only when peak >= this x touch_threshold
int  relative_release_samples = 2;   // dip must persist this many samples
```

Set both bools to `false` to get exactly the old behaviour back for comparison.

### Tuning, if needed

- Still merging (durations of 200 ms+, ILI values near 250 ms): raise
  `release_drop_percent` toward 55, or lower `plateau_rise_shift` to 6.
- Licks being split in two (durations under 20 ms, ILI near 60 ms): lower
  `release_drop_percent` to 30, raise `relative_release_samples` to 3, or raise
  `refractory_ms` from 25 to 40.
- `max_lick_ms` can now safely come down from 1500 to ~400. A real lick never
  exceeds ~150 ms, and a tighter watchdog recovers a genuinely stuck channel faster.

### Verifying on your rig

Set `serial_sensor_debug = 1`. Each lick now logs duration, peak, **inter-lick
interval**, and the DC level being removed:

```
LEFT  lick  dur=48 ms  peak=41  ILI=142 ms  dc=11  [split]  binTotal=27
```

- `dur` should cluster at 30–70 ms
- `ILI` should cluster at 100–160 ms (6–9 Hz)
- `dc` above zero during a bout is the offset that was breaking things
- `[split]` marks a lick recovered by relative release — i.e. one the old firmware
  would have swallowed

If ILI comes out near 250 or 300 ms, licks are still being merged. If it comes out
near 60 ms, they are being split.

The live `touch=YES/no` readout now carries a `dc=` column too.

---

# Part 2 — the interface

## Branding

`Logo.c` is regenerated. All four bitmaps are new: a Sweeney Lab splash mark with a
droplet and lick-train motif, a compact header logo, and a redrawn gear icon. Nothing
from the original lab remains. Dimensions are unchanged, so every `drawBitmap()` call
site still matches.

Splash reads **SWEENEY LAB / LIQ HD / LICKOMETER / by YD**, with a version line
underneath.

## Full colour customisation

The stock interface hard-coded four colours across roughly 300 call sites, with red
as the page background. Every one now routes through a named theme constant, defined
in one block in `LIQ_HD_Arduino_Mega.ino`:

```cpp
#define THEME_BG        0x0884   // page background
#define THEME_SURFACE   0x1948   // panels, buttons, header bar
#define THEME_TEXT      0xEF9F   // primary text and icons
#define THEME_ACCENT    0x2E98   // highlight and pressed state
#define THEME_MUTED     0x8D17   // secondary labels
#define THEME_LEFT      0x4D5F   // left bottle
#define THEME_RIGHT     0xFDA9   // right bottle
#define THEME_WARN      0xFAEA   // errors
```

Change one value, and it changes everywhere. Two alternative palettes ship commented
out: **Warm Dark**, and **High Contrast** for a dim room or dark cycle. Colours are
RGB565; there's a picker at rinkydinkelectronics.com/calc_rgb565.php, or compute it
directly with `((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3)`.

Also configurable:

```cpp
#define LAB_NAME     "SWEENEY LAB"
#define DEVICE_NAME  "LIQ HD"
#define BUILD_BY     "by YD"
#define FIRMWARE_VERSION "LIQ HD  v2.1  Sweeney Lab  by YD"
bool colour_code_bottles = true;   // false = both bottles plain white
```

## Recording screen

Rebuilt as two panels rather than four lines of text:

- A coloured spine identifies each bottle at a glance — blue left, amber right
- **Live contact dot**, filled while the tongue is actually on the spout. You can
  now watch detection happen without a laptop attached, which is useful when you're
  standing at the rig checking a bottle.
- `IN BOUT` appears while a bout is active
- Counts in 24 pt, right-aligned so digits don't jump as they grow
- A status strip showing bin length, current thresholds and the timeout count —
  so the settings that matter are visible during a run instead of buried

---

## Install

All six files, replacing what's in your folder:

```
LIQ_HD_Arduino_Mega.ino    LIQ_functions.ino    Pages.ino
SD_RTC_functions.ino       Screen_functions.ino  Logo.c
```

`Logo.c` **must** be replaced along with the rest — the previous one lacks nothing
functionally, but the new screens assume the new artwork dimensions.

Your tuning is preserved: `touch_threshold = 5`, `release_threshold = 2`,
`auto_threshold = false`. `LICK_TUNER` is unchanged and still lives in its own folder.

Same caveat as before: this compiles clean and the detector logic is verified in
simulation, but simulation is not your rig. Before it goes near real data, run a
session with `serial_sensor_debug = 1` and check that the duration and ILI
distributions look physiological — and if you can, validate a few minutes against
hand-scored video. A detector that counts too many licks fails much more quietly than
one that counts too few.
