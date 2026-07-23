 # LIQ HD lick detection — what was wrong and what changed

Your symptom — "sensitive enough to see licks means phantom licks, quiet enough to
stop phantoms means no licks" — is what you get when the signal and the noise are
roughly the same size. When that's true, no threshold works, because there is no
number that sits above one and below the other. Threshold tuning can't fix it.

There were three concrete bugs making that overlap far worse than it needs to be,
plus a signal-amplitude problem. All four are addressed here.

---

## 1. The I2C bus was running at 31 kHz, not 3.4 MHz

`SD_RTC_functions.ino` called:

```cpp
Wire.setClock(3400000);
```

The AVR Wire library computes `TWBR = ((F_CPU / frequency) - 16) / 2` in unsigned
arithmetic. On a 16 MHz Mega, `16000000 / 3400000` truncates to `4`, then `4 - 16`
wraps around, and the low byte lands on `TWBR = 250`. The resulting bus speed is:

```
SCL = 16000000 / (16 + 2*250) = 31,008 Hz
```

That's three times *slower* than standard 100 kHz mode. Combined with the recording
loop calling `rtc.now()` three times and `ts.touched()` once per iteration, the lick
sampling rate was roughly **50 Hz**. A mouse tongue contact lasts about 30–50 ms, and
the old code required two consecutive reads above threshold, so a lick had to last
40 ms just to be *eligible*. You were missing licks for reasons that had nothing to
do with sensitivity.

Fixed: 400 kHz (real fast mode), RTC reads cached at 4 Hz, touchscreen polled at
20 Hz. Sampling is now a steady 250 Hz.

## 2. The raw detector was reading a signal quantised to steps of 4

The raw mode computed:

```cpp
cap.baselineData(e) - cap.filteredData(e)
```

`baselineData()` in the Adafruit library reads an **8-bit** register and shifts it
left by 2:

```cpp
uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
return (bl << 2);
```

So that difference can only ever move in increments of 4 counts. Setting
`raw_touch_threshold = 1` (or 2, or 3) meant the detector was firing on the baseline
register ticking over by one LSB — pure quantisation, not your mouse.

Fixed: detection runs on `filteredData` directly, which is the full 10-bit value,
with a software baseline. Same signal, four times the resolution.

## 3. The release condition could essentially never be satisfied

```cpp
raw_use_absolute_change = true;
raw_release_threshold = 0;
// touch:   abs(change) >= 1
// release: abs(change) <= 0
```

Release required the reading to land *exactly* on the calibration value. Two things
follow. `abs()` means drift in the wrong direction counts as a touch, roughly
doubling the false-positive rate. And the channel only releases when the noise
happens to hit the idle value exactly — which, with a 4-count-quantised signal,
happens at random. The detector was effectively a noise-driven oscillator, which is
exactly what "lots of phantom licks" looks like.

Fixed: signed positive-only delta (a touch only ever pushes the reading *down*, so a
negative excursion is never a touch), plus proper Schmitt hysteresis — touch at `T`,
release at about 55% of `T`.

## 4. The signal itself was probably smaller than it needs to be

The MPR121 charges the electrode at constant current `CDC` for time `CDT` and
measures `V = I·t/C`. A lick adds capacitance `dC`, so the reading drops by roughly:

```
delta_counts  ≈  filtered_value × (dC / C)
```

Two things follow directly, and both are levers you weren't using:

- **A higher resting reading means every lick is worth more counts.** The firmware
  now sweeps CDC/CDT at startup and picks the setting that puts the resting value
  near 700 of 1023. This is free signal.
- **A larger baseline capacitance `C` shrinks every lick.** Long wire + large copper
  band + capacitive coupling through glass to a full tube of liquid all inflate `C`
  and dilute the signal. This one is a hardware fix — see below.

Also worth knowing: the MPR121 **ignores writes to almost every register while it is
in Run Mode**. Recent Adafruit library versions stop and restart the chip around each
write; older versions do not, and silently drop your settings. The new code stops the
chip itself and reads each register back to confirm the write landed, so it behaves
the same on any library version.

## 5. New: time-domain gating

This is what lets you keep a sensitive threshold without drowning in phantoms.

A mouse licks at 6–9 Hz. Each tongue contact lasts roughly 30–70 ms, with a 60–110 ms
gap. Electrical noise does not look like that — it is either much faster (spikes) or
much slower (drift). So the detector now requires a contact to be *held* for
`min_lick_ms` before it counts, enforces a `refractory_ms` lockout after each release,
and force-releases anything held longer than `max_lick_ms` as a stuck channel.

Blips that fail the duration test are counted in a rejection counter rather than
logged as licks, so you can see how hard the filter is working.

The software baseline also freezes for 2 s after any lick activity, so a drinking bout
can't pull the baseline up underneath itself, and tracks slowly (~8 s) otherwise so
overnight drift from a dropping liquid level is absorbed.

---

## Install

Back up your current folder first. Then copy these into
`~/Desktop/LIQ_HD/LIQ_HD_Arduino_Mega/`, replacing what's there:

```
LIQ_HD_Arduino_Mega.ino     (rewritten config block)
LIQ_functions.ino           (rewritten detector)
Pages.ino                   (loop timing fixes)
SD_RTC_functions.ino        (I2C clock fix)
Screen_functions.ino        (RTC polling fix)
```

`Logo.c` is unchanged and included only so the folder is complete.

Open `LIQ_HD_Arduino_Mega.ino` and upload. If your electrodes are not on E0/E1,
change `LEFT_ELECTRODE` / `RIGHT_ELECTRODE` at the top.

---

## Do this before you trust any threshold

`LICK_TUNER/LICK_TUNER.ino` is a standalone sketch. Upload it on its own, open the
Serial Monitor at **115200** with line ending set to **Newline**.

It runs the I2C scan, a CDC/CDT gain sweep and a noise measurement automatically,
then waits for commands. The one that matters is **`x`**:

> Type `x`, press Enter, and for 10 seconds touch each sipper with a **damp cotton
> bud** — not a whole finger. A finger is a far bigger electrode than a tongue and
> will flatter your setup.

It reports, per channel:

```
channel   peak delta   noise SD   signal/noise   verdict
```

That signal-to-noise number is the whole ballgame:

| S/N (cotton bud) | Meaning |
|---|---|
| 20 or more | Comfortable. A mouse tongue will land well clear of the noise. |
| 10–20 | Workable. |
| 5–10 | Marginal — expect misses, or phantoms, or both. |
| under 5 | No threshold will work. Fix the hardware. |

Other commands: `p` streams delta values for Tools → Serial Plotter (watch a real
lick go by), `l` logs each detected lick with its duration and peak, `s` re-runs the
gain sweep, `c` re-measures noise, `d` dumps registers.

---

## Hardware, in order of how much it will help

This is where the real gain is. The physics says signal scales with `dC/C`, so
shrinking `C` helps more than anything you can do in software.

1. **Connect to the metal sipper tube directly, not to copper tape on the glass.**
   Copper tape on the outside of a test tube couples to the liquid *through the
   glass* — roughly 1 mm of dielectric — which puts maybe 9 pF in series with your
   signal and attenuates it before it reaches the chip. A wire clipped or soldered
   straight onto the stainless sipper tube is galvanically connected to the liquid
   and gives you a much larger, much more stable delta. If you must stay with tape,
   make the patch as small as you can and put it where the liquid actually sits.

2. **Ground the animal.** The mouse completes the circuit through its body
   capacitance to ground. Tie the cage floor or grid to the Arduino's GND and the
   delta typically jumps several-fold. This is the change that most often turns a
   marginal rig into a comfortable one.

3. **Use a grounded power supply.** A laptop running on battery leaves the whole
   rig floating and the noise floor rises a lot. A grounded wall adapter is better
   than USB from a floating laptop.

4. **Shorten the electrode wire.** Every centimetre adds baseline capacitance, which
   directly divides your signal, and acts as an antenna for mains hum. Move the
   MPR121 board close to the bottles rather than running long leads to it. Don't use
   shielded cable — the shield adds far more capacitance than it saves in noise.

5. **Route the sensor wires away from the TFT ribbon and any mains cable,** and don't
   coil the slack.

6. **Watch for crosstalk between the two bottles.** In the plotter, if licking the
   left sipper produces a visible bump on the right channel too, the bottles are
   coupling through the animal. Separating the sippers physically is the fix; a
   higher threshold masks it but doesn't remove it.

Re-run the tuner's `x` test after each change. You should be able to watch the number
climb.

---

## Settings reference

All in `LIQ_HD_Arduino_Mega.ino`.

### Sensitivity

| Setting | Default | Notes |
|---|---|---|
| `auto_threshold` | `true` | Measures your actual idle noise at calibration and sets the threshold from it. Leave this on unless you have a reason. |
| `auto_threshold_sd_multiple` | `6.0` | Threshold in standard deviations of idle noise. Lower to 4–5 for more sensitivity, raise to 8 if phantoms persist. |
| `min_touch_threshold` | `6` | Floor. Below about 5 the chip chatters on its own. |
| `touch_threshold` | `12` | Only used when `auto_threshold = false`. Also editable on the touchscreen Settings page. |
| `release_percent` | `55` | Hysteresis. Raise toward 70 if a touch flickers; lower toward 40 if licks get split in two. |

### Time gating

| Setting | Default | Notes |
|---|---|---|
| `min_lick_ms` | `12` | Contact must be held this long to count. Raise to 20 to kill more noise; lower to 8 if you're clipping very brief contacts. This is your best anti-phantom knob. |
| `refractory_ms` | `25` | Lockout after a release. Mice can't exceed ~10 Hz, so up to 40 is safe. |
| `max_lick_ms` | `1500` | Anything longer is treated as a stuck channel and force-released. |

### Gain

| Setting | Default | Notes |
|---|---|---|
| `mpr_auto_gain` | `true` | Sweeps CDC/CDT at startup. Adds ~2 s when entering the recording page. |
| `mpr_target_filtered` | `700` | Resting reading to aim for, out of 1023. Higher = bigger licks, until it clips. |
| `mpr_ffi_code` | `2` | Samples averaged in the first filter: 0=6, 1=10, 2=18, 3=34. Higher is quieter. |

### Debug

`serial_sensor_debug`: `0` off, `1` one line per lick, `2` continuous stream for the
Serial Plotter. Set to `1` for a first overnight run — the log tells you the duration
and peak of every lick, and if the durations cluster around 30–70 ms you're seeing
real tongue contacts rather than noise.

---

## Suggested first session

1. Upload `LICK_TUNER`, run `x` with a damp cotton bud, note the S/N.
2. If it's under 10, work through the hardware list before going further.
3. Run `p` and watch the plotter while you dab the sipper. You should see clean
   rectangular pulses well above the threshold line.
4. Upload the main firmware with `serial_sensor_debug = 1`.
5. Run a 10-minute session with a real animal and read the lick durations. Real licks
   cluster tightly; noise doesn't.
6. Set `serial_sensor_debug = 0` for the real experiment.

One honest caveat: I've verified this compiles and the logic is right, but I can't
test it against your actual hardware. Validate against hand-scored video or a manual
count before you run a real experiment on it.
