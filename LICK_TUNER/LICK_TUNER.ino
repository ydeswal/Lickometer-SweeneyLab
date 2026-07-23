/******************************************************************
  LICK_TUNER - standalone MPR121 diagnostic for a LIQ HD lickometer

  Upload this on its own (not as part of the LIQ HD sketch), open the
  Serial Monitor at 115200 with line ending set to "Newline".

  It answers the one question threshold-fiddling cannot answer:

      how big is a real lick, in counts, compared to the noise floor?

  Once you know that number you stop guessing. If a lick is 10x the
  noise, almost any threshold works. If a lick is 1.5x the noise, no
  threshold works and the problem is the hardware, not the code.

  Commands (type a letter, press Enter):
      h   help
      i   I2C scan and chip health
      s   sweep charge current / charge time, pick the best gain
      c   calibrate: measure idle level and noise, recommend thresholds
      x   SIGNAL TEST: touch a sipper, get peak delta and signal-to-noise
      l   live lick detection log
      p   plotter stream (use Tools > Serial Plotter)
      d   dump MPR121 registers
      +   raise touch threshold by 1
      -   lower touch threshold by 1
      q   stop the current mode
******************************************************************/

#include <Wire.h>
#include <Adafruit_MPR121.h>

// ---------------- edit these to match your wiring ----------------
#define MPR_ADDRESS     0x5B
#define LEFT_ELECTRODE  0
#define RIGHT_ELECTRODE 1
// -----------------------------------------------------------------

// 400 kHz is correct fast mode on a 16 MHz AVR.
// NOTE: Wire.setClock(3400000) as used in stock LIQ HD wraps the TWBR
// calculation and actually gives you about 31 kHz. Do not use it.
#define I2C_CLOCK_HZ 400000UL

#define REG_TOUCHSTATUS 0x00
#define REG_OORSTATUS   0x02
#define REG_FILTDATA_0L 0x04
#define REG_BASELINE_0  0x1E
#define REG_TOUCHTH_0   0x41
#define REG_RELEASETH_0 0x42
#define REG_DEBOUNCE    0x5B
#define REG_CONFIG1     0x5C
#define REG_CONFIG2     0x5D
#define REG_ECR         0x5E
#define REG_AUTOCONFIG0 0x7B

Adafruit_MPR121 cap = Adafruit_MPR121();

const uint8_t CDT_CODES[] = { 1, 2, 3, 4, 5, 6, 7 };
const uint16_t CDT_NS[]   = { 500, 1000, 2000, 4000, 8000, 16000, 32000 };
const uint8_t CDC_VALUES[] = { 8, 16, 32, 63 };

int target_filtered = 700;
int ffi_code = 2;   // 18 samples
int sfi_code = 0;   // 4 samples
int esi_code = 0;   // 1 ms

int chosen_cdc = 16;
int chosen_cdt = 1;

const int NCH = 2;
const uint8_t ELECTRODE[NCH] = { LEFT_ELECTRODE, RIGHT_ELECTRODE };
const char *CHNAME[NCH] = { "LEFT ", "RIGHT" };

// detector settings, mirrored from the real firmware
int touch_threshold = 12;
int release_threshold = 6;
int min_lick_ms = 12;
int refractory_ms = 25;
int max_lick_ms = 1500;
int sample_interval_ms = 4;
int baseline_shift_slow = 11;
int baseline_shift_fast = 5;
int baseline_freeze_ms = 2000;
int fast_recover_delta = 25;

#define LK_Q 12
#define LK_IDLE 0
#define LK_CANDIDATE 1
#define LK_TOUCH 2

int32_t baseline_q[NCH];
int idle_level[NCH];
float noise_sd[NCH];
int noise_p2p[NCH];
uint8_t st[NCH];
unsigned long cand_start[NCH], touch_start[NCH], last_release[NCH], last_event[NCH];
int peak_delta[NCH];
int delta_now[NCH];
unsigned long lick_count[NCH], rejected[NCH];

char mode = 0;
unsigned long next_sample = 0;
bool calibrated = false;

//==================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) { }
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setWireTimeout(25000, true);

  Serial.println();
  Serial.println(F("================================================="));
  Serial.println(F("  LICK_TUNER"));
  Serial.println(F("================================================="));

  i2c_scan();

  if (!cap.begin(MPR_ADDRESS)) {
    Serial.println(F("!! MPR121 not found. Check address, wiring and pull-ups."));
    while (1) { }
  }
  Serial.print(F("MPR121 found at 0x"));
  Serial.println(MPR_ADDRESS, HEX);

  configure_chip();
  chip_health();
  sweep_gain();
  calibrate();
  help();
}

//==================================================================

void loop() {
  read_commands();

  if (mode == 'l') live_detect();
  else if (mode == 'p') plotter();
  else if (mode == 'x') signal_test_step();
}

//==================================================================
//  low level
//==================================================================

void mpr_stop() { cap.writeRegister(REG_ECR, 0x00); }

void mpr_run() {
  uint8_t highest = LEFT_ELECTRODE > RIGHT_ELECTRODE ? LEFT_ELECTRODE : RIGHT_ELECTRODE;
  cap.writeRegister(REG_ECR, 0x80 | (highest + 1));
}

bool wr(uint8_t reg, uint8_t val) {
  cap.writeRegister(reg, val);
  return cap.readRegister8(reg) == val;
}

int filt(uint8_t e) { return (int)cap.readRegister16(REG_FILTDATA_0L + e * 2); }

void apply_gain(int cdc, int cdt) {
  uint8_t c1 = ((uint8_t)(ffi_code & 3) << 6) | (uint8_t)(cdc & 0x3F);
  uint8_t c2 = ((uint8_t)(cdt & 7) << 5) | ((uint8_t)(sfi_code & 3) << 3) | (uint8_t)(esi_code & 7);
  mpr_stop();
  wr(REG_CONFIG1, c1);
  wr(REG_CONFIG2, c2);
  mpr_run();
  chosen_cdc = cdc;
  chosen_cdt = cdt;
}

void configure_chip() {
  mpr_stop();
  bool all_ok = true;
  all_ok &= wr(REG_DEBOUNCE, 0x00);
  all_ok &= wr(REG_AUTOCONFIG0, 0x00);
  for (uint8_t i = 0; i < 12; i++) {
    wr(REG_TOUCHTH_0 + 2 * i, 12);
    wr(REG_RELEASETH_0 + 2 * i, 6);
  }
  mpr_run();

  Serial.print(F("register writes verified: "));
  Serial.println(all_ok ? F("yes") : F("NO - your Adafruit_MPR121 library may be too old"));
}

void i2c_scan() {
  Serial.println(F("I2C scan:"));
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("   device at 0x"));
      Serial.print(a, HEX);
      if (a >= 0x5A && a <= 0x5D) Serial.print(F("   <- MPR121"));
      if (a == 0x68) Serial.print(F("   <- PCF8523 RTC"));
      if (a == 0x38) Serial.print(F("   <- FT6206 touch"));
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println(F("   nothing found - check SDA/SCL and power"));
}

void chip_health() {
  uint8_t stat_h = cap.readRegister8(0x01);
  uint16_t oor = cap.readRegister8(REG_OORSTATUS)
                 | ((uint16_t)cap.readRegister8(REG_OORSTATUS + 1) << 8);

  Serial.print(F("over-current flag (OVCF): "));
  Serial.println((stat_h & 0x80) ? F("SET - check the 75k REXT resistor / short on an electrode")
                                 : F("clear"));
  Serial.print(F("out-of-range flags: 0x"));
  Serial.println(oor, HEX);
}

//==================================================================
//  gain sweep
//==================================================================

void sweep_gain() {
  Serial.println();
  Serial.println(F("--- charge current / charge time sweep ---"));
  Serial.println(F("A lick is worth  filtered * (dC/C)  counts, so a higher resting"));
  Serial.println(F("reading means a bigger lick. Aiming for about 700 of 1023."));
  Serial.println();
  Serial.println(F("  CDC(uA)  CDT(us)   left   right"));

  int best_cdc = chosen_cdc, best_cdt = chosen_cdt;
  long best_score = 32767L;
  bool any = false;

  for (uint8_t ci = 0; ci < sizeof(CDC_VALUES); ci++) {
    for (uint8_t ti = 0; ti < sizeof(CDT_CODES); ti++) {
      apply_gain(CDC_VALUES[ci], CDT_CODES[ti]);
      delay(40);

      long sl = 0, sr = 0;
      for (uint8_t s = 0; s < 4; s++) {
        sl += filt(LEFT_ELECTRODE);
        sr += filt(RIGHT_ELECTRODE);
        delay(6);
      }
      int L = sl / 4, R = sr / 4;
      long avg = (L + R) / 2;

      Serial.print(F("   "));
      pad(CDC_VALUES[ci], 5);
      pad(CDT_NS[ti] / 1000, 8);
      if (CDT_NS[ti] < 1000) { Serial.print(F(".5")); } else { Serial.print(F("  ")); }
      pad(L, 6);
      pad(R, 7);

      if (avg > 980 || avg < 120) {
        Serial.println(F("   (out of range)"));
        continue;
      }
      long score = avg - target_filtered;
      if (score < 0) score = -score;
      if (score < best_score) { best_score = score; best_cdc = CDC_VALUES[ci]; best_cdt = CDT_CODES[ti]; any = true; }
      Serial.println();
    }
  }

  apply_gain(best_cdc, best_cdt);
  delay(60);

  Serial.println();
  if (!any) {
    Serial.println(F("!! No usable setting. Readings pinned at a rail."));
    Serial.println(F("   Pinned HIGH  -> electrode probably not connected."));
    Serial.println(F("   Pinned LOW   -> electrode capacitance far too large:"));
    Serial.println(F("                   shorten the wire, shrink the copper area."));
  } else {
    Serial.print(F("chosen: CDC="));
    Serial.print(best_cdc);
    Serial.print(F(" uA, CDT code "));
    Serial.print(best_cdt);
    Serial.print(F(" -> resting reading approx "));
    Serial.println((filt(LEFT_ELECTRODE) + filt(RIGHT_ELECTRODE)) / 2);
  }
}

void pad(long v, int width) {
  long a = (v < 0) ? -v : v;
  int len = (v < 0) ? 1 : 0;
  do { len++; a /= 10; } while (a);
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(v);
}

//==================================================================
//  calibration
//==================================================================

void calibrate() {
  Serial.println();
  Serial.println(F("--- idle noise measurement ---"));
  Serial.println(F("Hands off the rig for about 2 seconds..."));
  delay(1200);

  const int N = 300;
  for (int i = 0; i < NCH; i++) {
    int32_t sum = 0, sumsq = 0;
    int vmin = 32767, vmax = -32768;
    for (int s = 0; s < N; s++) {
      int v = filt(ELECTRODE[i]);
      sum += v;
      sumsq += (int32_t)v * (int32_t)v;
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
      delay(sample_interval_ms);
    }
    float mean = (float)sum / N;
    float var = ((float)sumsq - (float)sum * mean) / (N - 1);
    if (var < 0) var = 0;

    idle_level[i] = (int)(mean + 0.5);
    noise_sd[i] = sqrt(var);
    noise_p2p[i] = vmax - vmin;
    baseline_q[i] = (int32_t)idle_level[i] << LK_Q;
    st[i] = LK_IDLE;
    lick_count[i] = 0;
    rejected[i] = 0;

    Serial.print(CHNAME[i]);
    Serial.print(F(" E"));
    Serial.print(ELECTRODE[i]);
    Serial.print(F("  idle="));
    pad(idle_level[i], 5);
    Serial.print(F("  noiseSD="));
    Serial.print(noise_sd[i], 2);
    Serial.print(F("  peak-to-peak="));
    Serial.println(noise_p2p[i]);
  }

  float worst = noise_sd[0] > noise_sd[1] ? noise_sd[0] : noise_sd[1];
  int t = (int)(6.0 * worst + 0.5);
  if (t < 6) t = 6;
  if (t > 90) t = 90;
  touch_threshold = t;
  release_threshold = t * 55 / 100;
  if (release_threshold < 2) release_threshold = 2;

  Serial.print(F("recommended touch_threshold = "));
  Serial.print(touch_threshold);
  Serial.print(F("   release_threshold = "));
  Serial.println(release_threshold);

  if (worst > 8.0) {
    Serial.println();
    Serial.println(F("!! Idle noise is high. Before tuning anything in software:"));
    Serial.println(F("   - shorten the electrode wire, keep it away from the TFT ribbon"));
    Serial.println(F("   - power the Arduino from a grounded supply, not a battery laptop"));
    Serial.println(F("   - tie the cage floor / grid to Arduino GND"));
  }

  calibrated = true;
  Serial.println();
}

//==================================================================
//  signal test - the important one
//==================================================================

int st_peak[NCH];
unsigned long st_end = 0;

void signal_test_begin() {
  Serial.println();
  Serial.println(F("--- SIGNAL TEST ---"));
  Serial.println(F("For the next 10 seconds, touch the LEFT sipper with a wet"));
  Serial.println(F("fingertip, then the RIGHT one. Better still, wet a cotton bud"));
  Serial.println(F("and dab the spout - that is much closer to a mouse tongue"));
  Serial.println(F("than a whole finger, so it gives you an honest number."));
  Serial.println();
  for (int i = 0; i < NCH; i++) st_peak[i] = -32768;
  st_end = millis() + 10000UL;
}

void signal_test_step() {
  unsigned long t = millis();
  if ((long)(t - next_sample) < 0) return;
  next_sample = t + sample_interval_ms;

  for (int i = 0; i < NCH; i++) {
    int raw = filt(ELECTRODE[i]);
    int d = idle_level[i] - raw;
    if (d > st_peak[i]) st_peak[i] = d;
  }

  if ((long)(t - st_end) >= 0) {
    Serial.println(F("  channel   peak delta   noise SD   signal/noise   verdict"));
    for (int i = 0; i < NCH; i++) {
      float snr = noise_sd[i] > 0.01 ? (float)st_peak[i] / noise_sd[i] : 999.0;
      Serial.print(F("   "));
      Serial.print(CHNAME[i]);
      pad(st_peak[i], 12);
      Serial.print(F("      "));
      Serial.print(noise_sd[i], 2);
      Serial.print(F("        "));
      Serial.print(snr, 1);
      Serial.print(F("       "));
      if (snr >= 20) Serial.println(F("excellent"));
      else if (snr >= 10) Serial.println(F("good"));
      else if (snr >= 5) Serial.println(F("marginal - expect some misses"));
      else Serial.println(F("TOO LOW - fix the hardware, no threshold will work"));
    }
    Serial.println();
    Serial.println(F("A mouse tongue gives a smaller signal than your finger."));
    Serial.println(F("Aim for finger signal-to-noise of 20 or more so the tongue"));
    Serial.println(F("still lands comfortably above the noise."));
    Serial.println();
    mode = 0;
  }
}

//==================================================================
//  detector, identical logic to the real firmware
//==================================================================

void detector_step(bool log_licks) {
  unsigned long t = millis();
  if ((long)(t - next_sample) < 0) return;
  next_sample = t + sample_interval_ms;

  for (int i = 0; i < NCH; i++) {
    int raw = filt(ELECTRODE[i]);
    int base = (int)(baseline_q[i] >> LK_Q);
    int d = base - raw;
    delta_now[i] = d;

    switch (st[i]) {
      case LK_IDLE:
        if (d >= touch_threshold && (t - last_release[i]) >= (unsigned long)refractory_ms) {
          st[i] = LK_CANDIDATE;
          cand_start[i] = t;
          peak_delta[i] = d;
        }
        break;

      case LK_CANDIDATE:
        if (d > peak_delta[i]) peak_delta[i] = d;
        if (d < release_threshold) {
          st[i] = LK_IDLE;
          rejected[i]++;
        } else if ((t - cand_start[i]) >= (unsigned long)min_lick_ms) {
          st[i] = LK_TOUCH;
          touch_start[i] = cand_start[i];
          last_event[i] = t;
          lick_count[i]++;
        }
        break;

      case LK_TOUCH:
        if (d > peak_delta[i]) peak_delta[i] = d;
        if (d < release_threshold) {
          st[i] = LK_IDLE;
          last_release[i] = t;
          last_event[i] = t;
          if (log_licks) {
            Serial.print(CHNAME[i]);
            Serial.print(F("  lick #"));
            pad(lick_count[i], 5);
            Serial.print(F("   dur="));
            pad(t - touch_start[i], 4);
            Serial.print(F(" ms   peak="));
            pad(peak_delta[i], 4);
            Serial.print(F("   rejected blips so far="));
            Serial.println(rejected[i]);
          }
        } else if ((t - touch_start[i]) >= (unsigned long)max_lick_ms) {
          st[i] = LK_IDLE;
          last_release[i] = t;
          last_event[i] = t;
          baseline_q[i] = (int32_t)raw << LK_Q;
          if (log_licks) {
            Serial.print(CHNAME[i]);
            Serial.println(F("  stuck on, forced release and rebaseline"));
          }
        }
        break;
    }

    if (st[i] == LK_IDLE) {
      int32_t target = (int32_t)raw << LK_Q;
      if (d <= -fast_recover_delta) {
        baseline_q[i] += (target - baseline_q[i]) / (1L << baseline_shift_fast);
      } else if (d < release_threshold && (t - last_event[i]) >= (unsigned long)baseline_freeze_ms) {
        baseline_q[i] += (target - baseline_q[i]) / (1L << baseline_shift_slow);
      }
    }
  }
}

void live_detect() { detector_step(true); }

void plotter() {
  static unsigned long last_print = 0;
  detector_step(false);
  unsigned long t = millis();
  if (t - last_print < 20) return;
  last_print = t;
  // columns: leftDelta rightDelta touchTh releaseTh leftState rightState
  Serial.print(delta_now[0]); Serial.print('\t');
  Serial.print(delta_now[1]); Serial.print('\t');
  Serial.print(touch_threshold); Serial.print('\t');
  Serial.print(release_threshold); Serial.print('\t');
  Serial.print(st[0] == LK_TOUCH ? touch_threshold + 15 : 0); Serial.print('\t');
  Serial.println(st[1] == LK_TOUCH ? touch_threshold + 15 : 0);
}

//==================================================================

void dump_registers() {
  Serial.println(F("reg  value"));
  uint8_t regs[] = { 0x00, 0x01, 0x02, 0x03, 0x2B, 0x2C, 0x2D, 0x2E,
                     0x2F, 0x30, 0x31, 0x32, 0x41, 0x42, 0x43, 0x44,
                     0x5B, 0x5C, 0x5D, 0x5E, 0x7B };
  for (uint8_t i = 0; i < sizeof(regs); i++) {
    Serial.print(F("0x"));
    if (regs[i] < 16) Serial.print('0');
    Serial.print(regs[i], HEX);
    Serial.print(F("  0x"));
    uint8_t v = cap.readRegister8(regs[i]);
    if (v < 16) Serial.print('0');
    Serial.println(v, HEX);
  }
  Serial.print(F("baseline L/R: "));
  Serial.print(cap.readRegister8(REG_BASELINE_0 + LEFT_ELECTRODE) << 2);
  Serial.print('/');
  Serial.println(cap.readRegister8(REG_BASELINE_0 + RIGHT_ELECTRODE) << 2);
  Serial.print(F("filtered L/R: "));
  Serial.print(filt(LEFT_ELECTRODE));
  Serial.print('/');
  Serial.println(filt(RIGHT_ELECTRODE));
}

void help() {
  Serial.println();
  Serial.println(F("commands:  i=I2C scan  s=gain sweep  c=calibrate"));
  Serial.println(F("           x=SIGNAL TEST  l=live licks  p=plotter"));
  Serial.println(F("           d=register dump  + / - = threshold  q=stop  h=help"));
  Serial.print(F("current: touch="));
  Serial.print(touch_threshold);
  Serial.print(F("  release="));
  Serial.print(release_threshold);
  Serial.print(F("  CDC="));
  Serial.print(chosen_cdc);
  Serial.print(F("  CDT code="));
  Serial.println(chosen_cdt);
  Serial.println();
}

void read_commands() {
  if (!Serial.available()) return;
  char c = Serial.read();
  while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) Serial.read();

  switch (c) {
    case 'h': help(); break;
    case 'i': i2c_scan(); chip_health(); break;
    case 's': mode = 0; sweep_gain(); calibrate(); break;
    case 'c': mode = 0; calibrate(); break;
    case 'x': signal_test_begin(); mode = 'x'; break;
    case 'l':
      Serial.println(F("live lick log. press q then Enter to stop."));
      for (int i = 0; i < NCH; i++) { st[i] = LK_IDLE; lick_count[i] = 0; rejected[i] = 0; }
      mode = 'l';
      break;
    case 'p':
      Serial.println(F("leftDelta rightDelta touchTh releaseTh leftTouch rightTouch"));
      mode = 'p';
      break;
    case 'd': mode = 0; dump_registers(); break;
    case '+':
      touch_threshold++;
      release_threshold = touch_threshold * 55 / 100;
      Serial.print(F("touch=")); Serial.print(touch_threshold);
      Serial.print(F(" release=")); Serial.println(release_threshold);
      break;
    case '-':
      if (touch_threshold > 2) touch_threshold--;
      release_threshold = touch_threshold * 55 / 100;
      Serial.print(F("touch=")); Serial.print(touch_threshold);
      Serial.print(F(" release=")); Serial.println(release_threshold);
      break;
    case 'q': mode = 0; Serial.println(F("stopped.")); break;
    default: break;
  }
}
