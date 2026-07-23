//======================================================================================
//  LIQ HD - lick detection engine
//  One MPR121, one cage, two bottles.
//
//  Detection pipeline:
//    filteredData (full 10-bit)
//      -> software baseline with freeze-during-activity
//      -> signed positive delta (touch only ever pushes the reading DOWN)
//      -> Schmitt trigger, touch at T, release at ~55% of T
//      -> time gate: contact must last min_lick_ms, then refractory_ms lockout
//      -> stuck-channel watchdog
//      -> original LIQ HD bout logic, untouched
//======================================================================================

// MPR121 register addresses, named locally so this compiles against any library version.
#define LKREG_TOUCHSTATUS 0x00
#define LKREG_OORSTATUS   0x02
#define LKREG_FILTDATA_0L 0x04
#define LKREG_BASELINE_0  0x1E
#define LKREG_MHDR        0x2B
#define LKREG_NHDR        0x2C
#define LKREG_NCLR        0x2D
#define LKREG_FDLR        0x2E
#define LKREG_MHDF        0x2F
#define LKREG_NHDF        0x30
#define LKREG_NCLF        0x31
#define LKREG_FDLF        0x32
#define LKREG_NHDT        0x33
#define LKREG_NCLT        0x34
#define LKREG_FDLT        0x35
#define LKREG_TOUCHTH_0   0x41
#define LKREG_RELEASETH_0 0x42
#define LKREG_DEBOUNCE    0x5B
#define LKREG_CONFIG1     0x5C
#define LKREG_CONFIG2     0x5D
#define LKREG_ECR         0x5E
#define LKREG_AUTOCONFIG0 0x7B

#define LK_Q 12   // fixed point shift for the software baseline

//======================================================================================
//  Low level MPR121 access
//
//  The MPR121 ignores writes to almost every register while it is in Run Mode.
//  Recent Adafruit library versions stop and restart the chip around each write,
//  older ones do not. Stopping the chip ourselves works correctly either way.
//======================================================================================

static uint8_t lk_ecr_run_value() {
  uint8_t highest = LEFT_ELECTRODE;
  if (RIGHT_ELECTRODE > highest) highest = RIGHT_ELECTRODE;
  uint8_t n = highest + 1;
  if (n > 12) n = 12;
  return 0x80 | n;   // CL = 0b10 (baseline seeded from first reading), enable n electrodes
}

void mpr_stop() {
  cap.writeRegister(LKREG_ECR, 0x00);
}

void mpr_run() {
  cap.writeRegister(LKREG_ECR, lk_ecr_run_value());
}

// Write while stopped, then confirm it actually landed.
bool mpr_write_checked(uint8_t reg, uint8_t val) {
  cap.writeRegister(reg, val);
  return (cap.readRegister8(reg) == val);
}

int mpr_read_filtered(uint8_t electrode) {
  return (int)cap.readRegister16(LKREG_FILTDATA_0L + electrode * 2);
}

//======================================================================================
//  Charge current / charge time
//======================================================================================

static const uint8_t LK_CDT_CODES[] = { 1, 2, 3, 4, 5, 6, 7 };  // 0.5, 1, 2, 4, 8, 16, 32 us
static const uint8_t LK_CDC_VALUES[] = { 8, 16, 32, 63 };       // CDC * CDT is what matters

int mpr_chosen_cdc = 16;
int mpr_chosen_cdt = 1;

void mpr_apply_gain(int cdc, int cdt_code) {
  if (cdc < 1) cdc = 1;
  if (cdc > 63) cdc = 63;
  if (cdt_code < 1) cdt_code = 1;
  if (cdt_code > 7) cdt_code = 7;

  uint8_t cfg1 = ((uint8_t)(mpr_ffi_code & 0x03) << 6) | (uint8_t)(cdc & 0x3F);
  uint8_t cfg2 = ((uint8_t)(cdt_code & 0x07) << 5)
                 | ((uint8_t)(mpr_sfi_code & 0x03) << 3)
                 | (uint8_t)(mpr_esi_code & 0x07);

  mpr_stop();
  mpr_write_checked(LKREG_CONFIG1, cfg1);
  mpr_write_checked(LKREG_CONFIG2, cfg2);
  mpr_run();

  mpr_chosen_cdc = cdc;
  mpr_chosen_cdt = cdt_code;
}

// Sweep CDC/CDT and pick whatever puts the resting reading closest to the target.
// Bigger resting reading means every lick is worth more counts.
void mpr_auto_gain_sweep() {
  int best_cdc = mpr_manual_cdc;
  int best_cdt = mpr_manual_cdt_code;
  long best_score = 32767L;

  for (uint8_t ci = 0; ci < sizeof(LK_CDC_VALUES); ci++) {
    for (uint8_t ti = 0; ti < sizeof(LK_CDT_CODES); ti++) {
      mpr_apply_gain(LK_CDC_VALUES[ci], LK_CDT_CODES[ti]);
      delay(40);   // let the filters refill after the stop/run cycle

      long sum = 0;
      for (uint8_t s = 0; s < 4; s++) {
        sum += mpr_read_filtered(LEFT_ELECTRODE);
        sum += mpr_read_filtered(RIGHT_ELECTRODE);
        delay(6);
      }
      long avg = sum / 8;

      if (avg > 980 || avg < 120) continue;   // pinned at a rail, unusable

      long score = avg - (long)mpr_target_filtered;
      if (score < 0) score = -score;

      if (score < best_score) {
        best_score = score;
        best_cdc = LK_CDC_VALUES[ci];
        best_cdt = LK_CDT_CODES[ti];
      }
    }
  }

  mpr_apply_gain(best_cdc, best_cdt);
  delay(60);
}

//======================================================================================
//  Full sensor configuration. Called from the recording page and from auto calibration.
//======================================================================================

void set_sensor_settings() {
  mpr_stop();

  // Hardware thresholds only affect cap.touched(), which we keep as a cross-check.
  // Anything below about 5 chatters on its own, so clamp it.
  uint8_t hw_touch = (uint8_t)constrain(touch_threshold, 5, 255);
  uint8_t hw_release = (uint8_t)constrain(release_threshold, 2, hw_touch - 1);
  for (uint8_t i = 0; i < 12; i++) {
    mpr_write_checked(LKREG_TOUCHTH_0 + 2 * i, hw_touch);
    mpr_write_checked(LKREG_RELEASETH_0 + 2 * i, hw_release);
  }

  mpr_write_checked(LKREG_DEBOUNCE, (uint8_t)constrain(debounce, 0, 7));

  // Baseline filter. Rising side deliberately slow so the chip's own baseline does
  // not chase a sustained lick; falling side quick so it recovers after a bottle change.
  mpr_write_checked(LKREG_MHDR, 0x01);
  mpr_write_checked(LKREG_NHDR, 0x01);
  mpr_write_checked(LKREG_NCLR, 0x10);
  mpr_write_checked(LKREG_FDLR, 0x20);

  mpr_write_checked(LKREG_MHDF, 0x01);
  mpr_write_checked(LKREG_NHDF, 0x01);
  mpr_write_checked(LKREG_NCLF, 0x10);
  mpr_write_checked(LKREG_FDLF, 0x20);

  mpr_write_checked(LKREG_NHDT, 0x01);
  mpr_write_checked(LKREG_NCLT, 0x10);
  mpr_write_checked(LKREG_FDLT, 0x20);

  // Disable hardware autoconfiguration; we do our own sweep and we want the
  // settings to stay put across stop/run cycles.
  mpr_write_checked(LKREG_AUTOCONFIG0, 0x00);

  mpr_run();

  if (mpr_auto_gain) mpr_auto_gain_sweep();
  else mpr_apply_gain(mpr_manual_cdc, mpr_manual_cdt_code);
}

//======================================================================================
//  Calibration: measure the idle level and the idle noise, then set thresholds from it.
//======================================================================================

void calibrate_pair_sensors() {
  if (serial_sensor_debug > 0) {
    Serial.println();
    Serial.println(F("Calibrating. Do not touch the bottles or wires."));
  }

  delay(cal_settle_ms);

  for (uint8_t i = 0; i < LK_NCH; i++) {
    uint8_t e = (i == 0) ? LEFT_ELECTRODE : RIGHT_ELECTRODE;

    int32_t sum = 0;
    int32_t sumsq = 0;
    int vmin = 32767;
    int vmax = -32768;

    for (int s = 0; s < cal_samples; s++) {
      int v = mpr_read_filtered(e);
      sum += v;
      sumsq += (int32_t)v * (int32_t)v;
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
      delay(lick_sample_interval_ms);
    }

    float mean = (float)sum / (float)cal_samples;
    float var = ((float)sumsq - (float)sum * mean) / (float)(cal_samples - 1);
    if (var < 0) var = 0;

    lk_idle_level[i] = (int)(mean + 0.5);
    lk_noise_sd[i] = sqrt(var);
    lk_baseline_q[i] = (int32_t)lk_idle_level[i] << LK_Q;

    lk_state[i] = LK_IDLE;
    lk_cand_start[i] = 0;
    lk_touch_start[i] = 0;
    lk_last_release[i] = 0;
    lk_last_event[i] = 0;
    lk_peak_delta[i] = 0;
    lk_delta[i] = 0;
    lk_plateau_q[i] = 0;
    lk_plateau[i] = 0;
    lk_dc_raw[i] = 0;
    lk_relrel_count[i] = 0;
    lk_last_onset[i] = 0;

    if (serial_sensor_debug > 0) {
      Serial.print(i == 0 ? F("LEFT  E") : F("RIGHT E"));
      Serial.print(e);
      Serial.print(F("  idle="));
      Serial.print(lk_idle_level[i]);
      Serial.print(F("  noiseSD="));
      Serial.print(lk_noise_sd[i], 2);
      Serial.print(F("  p2p="));
      Serial.println(vmax - vmin);
    }
  }

  if (auto_threshold) {
    float worst = (lk_noise_sd[0] > lk_noise_sd[1]) ? lk_noise_sd[0] : lk_noise_sd[1];
    int t = (int)(auto_threshold_sd_multiple * worst + 0.5);
    if (t < min_touch_threshold) t = min_touch_threshold;
    if (t > max_touch_threshold) t = max_touch_threshold;
    touch_threshold = t;
    release_threshold = (int)((long)t * release_percent / 100L);
    if (release_threshold < 2) release_threshold = 2;
    if (release_threshold >= touch_threshold) release_threshold = touch_threshold - 1;
  }

  pair_raw_calibrated = true;

  if (serial_sensor_debug > 0) {
    Serial.print(F("gain: CDC="));
    Serial.print(mpr_chosen_cdc);
    Serial.print(F("uA  CDT code="));
    Serial.print(mpr_chosen_cdt);
    Serial.print(F("   thresholds: touch="));
    Serial.print(touch_threshold);
    Serial.print(F(" release="));
    Serial.println(release_threshold);
    Serial.println(F("Calibration done."));
  }
}

//======================================================================================
//  Bout bookkeeping. This is the original LIQ HD logic, moved into two functions so
//  the detector can call it with a true onset timestamp.
//======================================================================================

void lick_onset(uint8_t k, unsigned long onset_ms) {
  cal_timer = millis();
  time_now[k] = onset_ms;
  licking[k] = true;
  LickNumber[k] += 1;
  lick_bout_countdown[k] += 1;

  if (lick_bout_countdown[k] == 1) bout_start_timer[k] = onset_ms;

  if (lick_bout_countdown[k] == 3 && onset_ms - bout_start_timer[k] <= 1000) {
    BoutNumber[k] += 1;
    BoutLickNumber[k] += 2;
    bout_timer[k] = onset_ms;
    in_bout[k] = true;
  }

  if (in_bout[k]) {
    BoutLickNumber[k] += 1;
    bout_timer[k] = onset_ms;
  }
}

void lick_offset(uint8_t k, unsigned long release_ms) {
  last_lick_time[k] = release_ms;
  Elapsedtime[k] = release_ms - time_now[k];
  LickDuration[k] += Elapsedtime[k];
  BoutLickDuration[k] += Elapsedtime[k];

  if (in_bout[k] && lick_bout_countdown[k] == 3) BoutLickDuration_bytime[k] = BoutLickDuration[k];
  if (in_bout[k] && lick_bout_countdown[k] != 3) BoutLickDuration_bytime[k] += Elapsedtime[k];

  licking[k] = false;
}

//======================================================================================
//  Live  touch=YES/no  readout.
//  Entirely removed from the build if SHOW_TOUCH_STATE is commented out in the
//  main tab.
//======================================================================================

#ifdef SHOW_TOUCH_STATE

bool ts_prev_touch[LK_NCH] = { false, false };
bool ts_first_line = true;
unsigned long ts_last_print_ms = 0;

// right-align a number so the columns stay readable as values change width
void ts_pad(long v, int width) {
  long a = (v < 0) ? -v : v;
  int len = (v < 0) ? 1 : 0;
  do { len++; a /= 10; } while (a);
  for (int i = len; i < width; i++) Serial.print(' ');
  Serial.print(v);
}

void print_touch_state() {
  if (!touch_state_enabled) return;

  // serial_sensor_debug = 2 is the Serial Plotter stream. Mixing text into it would
  // garble the plot, so stay out of the way in that mode.
  if (serial_sensor_debug >= 2) return;

  bool now_touch[LK_NCH];
  bool changed = false;
  for (uint8_t i = 0; i < LK_NCH; i++) {
    now_touch[i] = (lk_state[i] == LK_TOUCH);
    if (now_touch[i] != ts_prev_touch[i]) changed = true;
  }

  unsigned long t = millis();

  if (touch_state_on_change_only) {
    if (!changed && !ts_first_line) return;
  } else {
    if (!ts_first_line && (t - ts_last_print_ms) < (unsigned long)touch_state_interval_ms) return;
  }

  ts_last_print_ms = t;
  for (uint8_t i = 0; i < LK_NCH; i++) ts_prev_touch[i] = now_touch[i];

  if (ts_first_line) {
    ts_first_line = false;
    Serial.println();
    Serial.println(F("--- live touch readout ---"));
    Serial.print(F("delta = how far below baseline the reading is, / = touch threshold ("));
    Serial.print(touch_threshold);
    Serial.println(F(")"));
    Serial.println(F("dc = sustained offset removed by the plateau tracker"));
    Serial.print(F("(testing) = above threshold, waiting out the "));
    Serial.print(min_lick_ms);
    Serial.println(F(" ms hold before it counts"));
    Serial.println();
  }

  ts_pad((long)t, 8);
  Serial.print(F(" ms   "));

  for (uint8_t i = 0; i < LK_NCH; i++) {
    Serial.print(i == 0 ? F("LEFT touch=") : F("RIGHT touch="));
    Serial.print(now_touch[i] ? F("YES") : F("no "));

    if (touch_state_show_numbers) {
      Serial.print(F("  delta="));
      ts_pad((long)lk_delta[i], 4);
      Serial.print(F(" /"));
      ts_pad((long)touch_threshold, 3);
      // dc = the sustained offset the plateau tracker is removing. If this sits
      // well above zero during a bout, that is the merging problem being handled.
      Serial.print(F("  dc="));
      ts_pad((long)lk_plateau[i], 3);
      // a channel part way through the min_lick_ms hold test
      if (lk_state[i] == LK_CANDIDATE) Serial.print(F(" (testing)"));
      else Serial.print(F("          "));
    }

    if (i == 0) Serial.print(F("   |   "));
  }

  Serial.println();
}

#endif  // SHOW_TOUCH_STATE

//======================================================================================
//  The detector itself.
//======================================================================================

void lick_sample_and_update() {
  unsigned long t = millis();
  if ((long)(t - lk_next_sample_ms) < 0) return;
  lk_next_sample_ms = t + (unsigned long)lick_sample_interval_ms;

  bool stream = (serial_sensor_debug >= 2)
                && (t - last_serial_sensor_debug_ms >= (unsigned long)serial_debug_interval_ms);

  for (uint8_t i = 0; i < LK_NCH; i++) {
    uint8_t e = (i == 0) ? LEFT_ELECTRODE : RIGHT_ELECTRODE;

    int raw = mpr_read_filtered(e);
    lk_filtered[i] = (int16_t)raw;

    int base = (int)(lk_baseline_q[i] >> LK_Q);
    int d_dc = base - raw;           // positive = added capacitance = something touching
    lk_dc_raw[i] = (int16_t)d_dc;

    // ---- plateau tracker: estimate and remove the sustained DC component ----
    //
    // Rises slowly so a 30-70 ms lick is not tracked out; falls quickly so the floor
    // between two contacts is re-acquired well inside a 60-110 ms inter-lick gap.
    // Frozen while a contact is confirmed, so it cannot chase the lick itself and
    // corrupt the measured duration.
    int d = d_dc;
    if (use_plateau_tracker) {
      if (lk_state[i] != LK_TOUCH) {
        int32_t target = (int32_t)d_dc << LK_Q;
        int shift = (target < lk_plateau_q[i]) ? plateau_fall_shift : plateau_rise_shift;
        lk_plateau_q[i] += (target - lk_plateau_q[i]) >> shift;

        int32_t hi = (int32_t)plateau_max << LK_Q;
        if (lk_plateau_q[i] > hi) lk_plateau_q[i] = hi;
        if (lk_plateau_q[i] < 0) lk_plateau_q[i] = 0;
      }
      lk_plateau[i] = (int16_t)(lk_plateau_q[i] >> LK_Q);
      d = d_dc - lk_plateau[i];
    } else {
      lk_plateau_q[i] = 0;
      lk_plateau[i] = 0;
    }

    lk_delta[i] = (int16_t)d;

    int Ton = touch_threshold;
    int Toff = release_threshold;

    switch (lk_state[i]) {

      case LK_IDLE:
        if (d >= Ton && (t - lk_last_release[i]) >= (unsigned long)refractory_ms) {
          lk_state[i] = LK_CANDIDATE;
          lk_cand_start[i] = t;
          lk_peak_delta[i] = (int16_t)d;
          lk_relrel_count[i] = 0;
        }
        break;

      case LK_CANDIDATE:
        if (d > lk_peak_delta[i]) lk_peak_delta[i] = (int16_t)d;
        if (d < Toff) {
          // Collapsed before it lasted long enough to be a tongue. This is the
          // filter that removes electrical noise without desensitising the rig.
          lk_state[i] = LK_IDLE;
          lk_rejected[i]++;
        } else if ((t - lk_cand_start[i]) >= (unsigned long)min_lick_ms) {
          lk_state[i] = LK_TOUCH;
          lk_touch_start[i] = lk_cand_start[i];
          lk_last_event[i] = t;
          lick_onset(i, lk_cand_start[i]);
        }
        break;

      case LK_TOUCH: {
        if (d > lk_peak_delta[i]) lk_peak_delta[i] = (int16_t)d;

        bool absolute_release = (d < Toff);

        // ---- relative release ----
        // A tongue withdrawal is a large fractional drop from the peak of that
        // contact, even when the signal never reaches the absolute floor. This is
        // what separates licks that would otherwise merge into one long touch.
        bool relative_release = false;
        if (use_relative_release && !absolute_release) {
          bool peak_big_enough = (lk_peak_delta[i] >= (int16_t)(relative_release_min_peak * Ton));
          long drop_level = ((long)lk_peak_delta[i] * (long)release_drop_percent) / 100L;
          if (drop_level < (long)Toff) drop_level = (long)Toff;

          if (peak_big_enough && (long)d < drop_level
              && (t - lk_touch_start[i]) >= (unsigned long)min_lick_ms) {
            if (lk_relrel_count[i] < 255) lk_relrel_count[i]++;
            if (lk_relrel_count[i] >= (uint8_t)relative_release_samples) relative_release = true;
          } else {
            lk_relrel_count[i] = 0;
          }
        }

        if (absolute_release || relative_release) {
          lk_state[i] = LK_IDLE;
          lk_last_release[i] = t;
          lk_last_event[i] = t;
          lk_relrel_count[i] = 0;
          if (relative_release) lk_merged_saved[i]++;

          // Seed the plateau at the current level so the next contact in the bout is
          // measured from the floor it actually sits on, not from the idle level.
          if (use_plateau_tracker && relative_release) {
            lk_plateau_q[i] = (int32_t)lk_dc_raw[i] << LK_Q;
          }

          lick_offset(i, t);

          if (serial_sensor_debug == 1) {
            Serial.print(i == 0 ? F("LEFT  lick  dur=") : F("RIGHT lick  dur="));
            Serial.print(t - lk_touch_start[i]);
            Serial.print(F(" ms  peak="));
            Serial.print(lk_peak_delta[i]);
            Serial.print(F("  ILI="));
            Serial.print(lk_last_onset[i] ? (lk_touch_start[i] - lk_last_onset[i]) : 0);
            Serial.print(F(" ms  dc="));
            Serial.print(lk_plateau[i]);
            Serial.print(relative_release ? F("  [split]") : F("        "));
            Serial.print(F("  binTotal="));
            Serial.println(LickNumber[i]);
          }
          lk_last_onset[i] = lk_touch_start[i];

        } else if ((t - lk_touch_start[i]) >= (unsigned long)max_lick_ms) {
          // Held far too long to be a lick. Something is shorted, wet or drifting.
          // Release it and re-seed the baseline so the channel recovers by itself.
          lk_state[i] = LK_IDLE;
          lk_last_release[i] = t;
          lk_last_event[i] = t;
          lk_relrel_count[i] = 0;
          lick_offset(i, t);
          lk_baseline_q[i] = (int32_t)raw << LK_Q;
          lk_plateau_q[i] = 0;
          lk_stuck[i]++;
          if (serial_sensor_debug >= 1) {
            Serial.print(i == 0 ? F("LEFT") : F("RIGHT"));
            Serial.println(F(" stuck-on, forced release and rebaseline"));
          }
        }
        break;
      }
    }

    // ---- software baseline (slow, absolute) ----
    // Operates on the un-subtracted signal, so it still represents the true idle level.
    if (lk_state[i] == LK_IDLE) {
      int32_t target = (int32_t)raw << LK_Q;
      if (d_dc <= -fast_recover_delta) {
        // Reading is far ABOVE the baseline: capacitance dropped. A bottle was
        // pulled, a wire came loose, or the tube was refilled. Re-acquire quickly.
        lk_baseline_q[i] += (target - lk_baseline_q[i]) / (1L << baseline_shift_fast);
        lk_plateau_q[i] = 0;
      } else if (d_dc < Toff && (t - lk_last_event[i]) >= (unsigned long)baseline_freeze_ms) {
        // Quiet and well below threshold: track slow environmental drift only.
        lk_baseline_q[i] += (target - lk_baseline_q[i]) / (1L << baseline_shift_slow);
      }
    }

    if (stream) {
      Serial.print(lk_delta[i]);
      Serial.print('\t');
      Serial.print(lk_plateau[i]);
      Serial.print('\t');
      Serial.print(lk_state[i] == LK_TOUCH ? touch_threshold + 10 : 0);
      Serial.print('\t');
    }
  }

  if (stream) {
    Serial.print(touch_threshold);
    Serial.print('\t');
    Serial.println(release_threshold);
    last_serial_sensor_debug_ms = t;
  }

#ifdef SHOW_TOUCH_STATE
  print_touch_state();
#endif
}

//======================================================================================
//  Called from the recording loop.
//======================================================================================

void Record_Licks() {
  if (!pair_raw_calibrated) calibrate_pair_sensors();

  lk_active_ms = millis();   // tells wait() that the recording loop is live

  lick_sample_and_update();

  // keep the hardware touch flags available for cross-checking / the stock display
  currtouched1 = 0;
  if (lk_state[0] == LK_TOUCH) currtouched1 |= _BV(LEFT_ELECTRODE);
  if (lk_state[1] == LK_TOUCH) currtouched1 |= _BV(RIGHT_ELECTRODE);
  lasttouched1 = currtouched1;

  for (int k = 0; k < ACTIVE_SIPPER_COUNT; k++) {
    if (lick_bout_countdown[k] <= 2 && millis() - bout_start_timer[k] >= 1000) {
      bout_start_timer[k] = 0;
      lick_bout_countdown[k] = 0;
      if (log_by_bout) {
        BoutLickNumber[k] = 0;
        BoutLickDuration[k] = 0;
      }
    }

    if (lick_bout_countdown[k] >= 3 && millis() - bout_timer[k] >= (unsigned long)bout_cutoff) {
      in_bout[k] = false;

      if (log_by_bout) {
        BoutDuration[k] = last_lick_time[k] - bout_start_timer[k];
        if (BoutDuration[k] > 0) LickFrequency[k] = float(BoutLickNumber[k]) / (float(BoutDuration[k]) / 1000.0);
        else LickFrequency[k] = 0;
        if (BoutLickNumber[k] > 0) ILI[k] = (float(BoutDuration[k]) - float(BoutLickDuration[k])) / float(BoutLickNumber[k]);
        else ILI[k] = 0;

        write_to_file_by_bout(k);
        calc_total_LN();

        BoutDuration[k] = 0;
        BoutLickNumber[k] = 0;
        BoutLickDuration[k] = 0;
        BoutLickDuration_bytime[k] -= snap_BoutLickDuration_bytime[k];
      }

      if (log_by_time) BoutDuration[k] = BoutDuration[k] + (last_lick_time[k] - bout_start_timer[k]);

      bout_start_timer[k] = 0;
      lick_bout_countdown[k] = 0;
    }
  }
}

//======================================================================================

//======================================================================================
//  Lossless time-bin handover.
//
//  The old sequence was: stop sampling -> write the row -> zero the counters.
//  Anything the animal did during that window was either never detected or was
//  zeroed before it reached the card. On a 1 minute bin that window includes an
//  SD.begin() health check and, every SYNC_INTERVAL, a real FAT flush.
//
//  Instead: snapshot the counters, log the snapshot while continuing to sample,
//  then SUBTRACT the snapshot rather than zeroing. Licks that land mid-write stay
//  in the counters and are carried into the next bin instead of being discarded.
//======================================================================================

void snapshot_counters() {
  for (uint8_t k = 0; k < LK_NCH; k++) {
    snap_LickNumber[k] = LickNumber[k];
    snap_LickDuration[k] = LickDuration[k];
    snap_BoutNumber[k] = BoutNumber[k];
    snap_BoutDuration[k] = BoutDuration[k];
    snap_BoutLickNumber[k] = BoutLickNumber[k];
    snap_BoutLickDuration[k] = BoutLickDuration[k];
    snap_BoutLickDuration_bytime[k] = BoutLickDuration_bytime[k];
  }
}

void subtract_snapshot() {
  for (uint8_t k = 0; k < LK_NCH; k++) {
    LickNumber[k] -= snap_LickNumber[k];
    LickDuration[k] -= snap_LickDuration[k];
    BoutNumber[k] -= snap_BoutNumber[k];
    BoutDuration[k] -= snap_BoutDuration[k];
    BoutLickNumber[k] -= snap_BoutLickNumber[k];
    BoutLickDuration[k] -= snap_BoutLickDuration[k];
    BoutLickDuration_bytime[k] -= snap_BoutLickDuration_bytime[k];
    Elapsedtime[k] = 0;
  }
}

// Totals shown on screen come from what was actually written to the card.
void calc_total_LN_from_snapshot() {
  if (log_by_time) {
    for (uint8_t k = 0; k < LK_NCH; k++) total_LN[k] += snap_LickNumber[k];
  }
  if (log_by_bout) {
    for (uint8_t k = 0; k < LK_NCH; k++) total_LN[k] = LickNumber[k];
  }
}

// One call for the whole bin handover, so ordering cannot drift out of step.
void log_time_bin() {
  snapshot_counters();
  write_to_file_by_time();
  calc_total_LN_from_snapshot();
  subtract_snapshot();
}

//======================================================================================

void reset_total_LN() {
  for (int k = 0; k < 36; k++) total_LN[k] = 0;
}

//======================================================================================

void reset_variables() {
  for (int k = 0; k < 36; k++) {
    LickNumber[k] = 0;
    LickDuration[k] = 0;
    BoutNumber[k] = 0;
    BoutDuration[k] = 0;
    BoutLickNumber[k] = 0;
    BoutLickDuration[k] = 0;
    BoutLickDuration_bytime[k] -= snap_BoutLickDuration_bytime[k];
    LickFrequency[k] = 0;
    ILI[k] = 0;
    Elapsedtime[k] = 0;
    lick_bout_countdown[k] = 0;
    bout_timer[k] = 0;
    bout_start_timer[k] = 0;
    in_bout[k] = false;
    licking[k] = false;
    time_now[k] = 0;
    last_lick_time[k] = 0;
  }

  // A lick can straddle a time-bin boundary. Re-assert it so the release that is
  // still coming is attributed correctly instead of being counted as a fresh lick.
  unsigned long t = millis();
  for (uint8_t i = 0; i < LK_NCH; i++) {
    if (lk_state[i] == LK_TOUCH) {
      licking[i] = true;
      time_now[i] = t;
    }
  }
}

//======================================================================================

void update_sippers() {
  for (int k = 0; k < ACTIVE_SIPPER_COUNT; k++) {
    if (licking[k]) {
      Elapsedtime[k] = millis() - time_now[k];
      LickDuration[k] = LickDuration[k] + Elapsedtime[k];
      if (in_bout[k]) {
        BoutLickDuration[k] = BoutLickDuration[k] + Elapsedtime[k];
        BoutLickDuration_bytime[k] = BoutLickDuration_bytime[k] + Elapsedtime[k];
      }
    }
    if (in_bout[k]) BoutDuration[k] = BoutDuration[k] + (millis() - bout_start_timer[k]);
  }
}

//======================================================================================

void reset_time_now() {
  for (int k = 0; k < ACTIVE_SIPPER_COUNT; k++) {
    time_now[k] = millis();
    if (in_bout[k]) bout_start_timer[k] = last_lick_time[k];
  }
}

//======================================================================================

void calc_total_LN() {
  if (log_by_time) {
    for (int k = 0; k < ACTIVE_SIPPER_COUNT; k++) total_LN[k] = total_LN[k] + LickNumber[k];
  }
  if (log_by_bout) {
    for (int k = 0; k < ACTIVE_SIPPER_COUNT; k++) total_LN[k] = LickNumber[k];
  }
}

//======================================================================================
//  Blocking wait that still services the sensor, so licks are not lost during
//  screen redraws and button debounce delays.
//======================================================================================

void wait(unsigned long ms) {
  unsigned long X = millis();
  while (millis() - X < ms) {
    // Only keep sampling if the recording loop is actually running. The stock code
    // sets `recording = true` and never clears it, so we key off recent activity
    // instead: Record_Licks() stamps lk_active_ms every time it runs.
    if (pair_raw_calibrated && (millis() - lk_active_ms) < 300UL) lick_sample_and_update();
  }
}

//======================================================================================
//  Cached RTC minute. The stock loop called rtc.now() three times per iteration,
//  which on this I2C bus costs more time than the sensor read itself.
//======================================================================================

int cached_minute() {
  unsigned long t = millis();
  if (t - rtc_cache_ms >= (unsigned long)rtc_cache_interval_ms) {
    rtc_cache_ms = t;
    now = rtc.now();
  }
  return now.minute();
}

//======================================================================================
//  Rate limited touchscreen poll. Reading the FT6206 is the most expensive I2C
//  transaction in the recording loop and 20 Hz is plenty for a finger.
//======================================================================================

bool ts_poll() {
  static unsigned long last_ts = 0;
  unsigned long t = millis();
  if (t - last_ts < 50) return false;
  last_ts = t;
  return ts.touched();
}

//======================================================================================

void set_defaults() {
  default_lights_on = lights_on;
  default_lights_off = lights_off;
  default_touch_threshold = touch_threshold;
  default_release_threshold = release_threshold;
  default_LOG_INTERVAL = LOG_INTERVAL;
  dafault_SYNC_INTERVAL = SYNC_INTERVAL;
  default_auto_cal = auto_cal;
  default_auto_cal_time = auto_cal_time;
  default_auto_cal_sec_since_last_lick = auto_cal_sec_since_last_lick;
}

//======================================================================================

void auto_calibration() {
  if (!auto_cal) return;

  // rate limit: this used to hit the RTC on every single loop iteration
  static unsigned long last_check = 0;
  if (millis() - last_check < 1000) return;
  last_check = millis();

  unsigned long temp_cal_time = 1000UL * (unsigned long)auto_cal_sec_since_last_lick;
  unsigned long temp_cal_timer = millis() - cal_timer;
  now = rtc.now();
  if (auto_cal_flag && now.hour() == auto_cal_time && temp_cal_timer > temp_cal_time) {
    auto_cal_flag = !auto_cal_flag;
    cap.begin(ACTIVE_MPR121_ADDRESS);
    set_sensor_settings();
    pair_raw_calibrated = false;
    calibrate_pair_sensors();
  }
  if (now.hour() != auto_cal_time) auto_cal_flag = true;
}
