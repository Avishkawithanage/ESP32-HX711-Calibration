/*
 * ============================================================================
 *  SmartWaste Pro - STANDALONE 3-channel HX711 load cell calibration tool
 * ============================================================================
 *  Board     : ESP32 DevKit (Arduino framework, PlatformIO)
 *  Library   : bogde/HX711 (0.7.5)
 *  Serial    : 115200 baud, commands are single letters (see HELP below)
 *
 *  What this tool does
 *    - Checks that each HX711 is talking to the ESP32 (diagnostic, command D / A)
 *    - Measures a stable zero (tare) with NOTHING on the load cell (command T)
 *    - Measures the known 250 g weight and CALCULATES the calibration factor
 *      (command C, then G). Nothing is invented: factor comes only from readings.
 *    - Rejects the calibration (and prints no factor) if readings are noisy,
 *      the response is too small, the ADC is saturated, or communication fails.
 *    - Shows live weight in grams after calibration (command W).
 *
 *  Math (matches bogde/HX711 source code):
 *      get_value(n) = read_average(n) - OFFSET
 *      get_units(n) = get_value(n) / SCALE
 *      => grams = (raw - OFFSET) / SCALE
 *      => SCALE = (raw_with_known_weight - OFFSET) / known_weight_in_grams
 *    SCALE has units of "ADC counts per gram". It may be NEGATIVE (see guide).
 *
 *  Not used here on purpose: Wi-Fi, Firebase, Blynk, ultrasonic, buttons, LEDs, buzzer.
 * ============================================================================
 */
#include <Arduino.h>
#include <HX711.h>
#include <math.h>

// ============================== SETTINGS ====================================
static const uint8_t NUM_CH = 3;
static const char* const CH_NAME[NUM_CH] = { "Plastic", "Food", "Metal" };
static const uint8_t DT_PIN[NUM_CH]  = { 36, 4, 17 };   // HX711 DT / DOUT
static const uint8_t SCK_PIN[NUM_CH] = { 33, 16, 21 };  // HX711 SCK / CLK

static const float REF_WEIGHT_G = 500.0f;               // known calibration weight

// Sampling
static const uint32_t READY_TIMEOUT_MS   = 1000;  // max wait for ONE sample (10 SPS needs ~100 ms)
static const uint8_t  MAX_BLOCK          = 40;    // array size limit
static const uint8_t  SAMPLES_PER_BLOCK  = 20;    // ~2 s per block at 10 SPS
static const uint8_t  SETTLE_DISCARD     = 3;     // samples thrown away before each block
static const uint8_t  MAX_MISSED         = 3;     // timeouts allowed before a block is aborted

// Acceptance limits (raw ADC counts). Tune only if you understand why.
static const long   TARE_MAX_PP_COUNTS    = 2000;   // max peak-to-peak noise allowed during tare
static const long   MIN_DELTA_COUNTS      = 2000;   // min |loaded - unloaded| to trust the factor
static const float  MAX_PP_FRACTION       = 0.02f;  // noise (peak-to-peak) must be <= 2% of the delta
static const float  MAX_MISMATCH_FRACTION = 0.01f;  // block A and B must agree within 1% of delta
static const long   RAW_NEAR_LIMIT        = 8300000L; // HX711 range is -8388608..+8388607

// Live display
static const uint8_t  RING_SIZE         = 10;     // 10-sample moving average (like the main firmware)
static const uint32_t PRINT_INTERVAL_MS = 500;
static const uint32_t STALL_WARN_MS     = 2000;

// ============================== STATE =======================================
enum Mode { MODE_IDLE, MODE_WAIT_WEIGHT, MODE_STREAM_RAW, MODE_STREAM_WEIGHT };

static HX711 hx[NUM_CH];
static bool  tareDone[NUM_CH]  = { false, false, false };
static bool  calDone[NUM_CH]   = { false, false, false };
static long  tarePP[NUM_CH]    = { 0, 0, 0 };
static long  offsetVal[NUM_CH] = { 0, 0, 0 };
static float scaleVal[NUM_CH]  = { 0, 0, 0 };

static uint8_t active = 0;
static Mode    mode   = MODE_IDLE;

// live display state
static long     ring[RING_SIZE];
static uint8_t  ringCount = 0, ringIdx = 0;
static long     lastRaw = 0;
static uint32_t lastSampleMs = 0, lastPrintMs = 0;

struct BlockStats {
  bool    ok;
  uint8_t good;
  uint8_t missed;
  long    minRaw, maxRaw, pp;
  double  mean;     // trimmed mean (drops highest and lowest 10%)
  double  stddev;   // standard deviation of all samples
};

// ============================== HELPERS =====================================

// Wait (with a timeout) for ONE sample. Never waits forever.
// is_ready() is true when DT is LOW. Because we confirm readiness first,
// hx.read() will not block (read() itself would otherwise wait forever).
static bool readSample(uint8_t ch, long &raw) {
  uint32_t t0 = millis();
  while (!hx[ch].is_ready()) {
    if (millis() - t0 >= READY_TIMEOUT_MS) return false;
    delay(1);
  }
  raw = hx[ch].read();
  return true;
}

// Take 'discard' throw-away samples, then 'n' kept samples, and compute statistics.
static bool measureBlock(uint8_t ch, uint8_t discard, uint8_t n, BlockStats &st, bool printRaw) {
  st = BlockStats();
  if (n > MAX_BLOCK) n = MAX_BLOCK;
  if (n < 8) n = 8;
  long buf[MAX_BLOCK];
  long r = 0;
  uint8_t missed = 0;

  uint8_t got = 0;
  while (got < discard) {
    if (readSample(ch, r)) got++;
    else if (++missed > MAX_MISSED) { st.missed = missed; return false; }
  }
  got = 0;
  while (got < n) {
    if (readSample(ch, r)) buf[got++] = r;
    else if (++missed > MAX_MISSED) { st.missed = missed; st.good = got; return false; }
  }
  st.good = got;
  st.missed = missed;

  // sorted copy (insertion sort) for min / max / trimmed mean
  long s[MAX_BLOCK];
  for (uint8_t i = 0; i < n; i++) s[i] = buf[i];
  for (uint8_t i = 1; i < n; i++) {
    long key = s[i];
    int j = i - 1;
    while (j >= 0 && s[j] > key) { s[j + 1] = s[j]; j--; }
    s[j + 1] = key;
  }
  st.minRaw = s[0];
  st.maxRaw = s[n - 1];
  st.pp = st.maxRaw - st.minRaw;

  uint8_t trim = n / 10;
  double sum = 0;
  for (uint8_t i = trim; i < n - trim; i++) sum += (double)s[i];
  st.mean = sum / (double)(n - 2 * trim);

  double plain = 0;
  for (uint8_t i = 0; i < n; i++) plain += (double)buf[i];
  plain /= n;
  double var = 0;
  for (uint8_t i = 0; i < n; i++) { double d = (double)buf[i] - plain; var += d * d; }
  st.stddev = sqrt(var / (double)(n - 1));

  if (printRaw) {
    Serial.print("  raw samples:");
    for (uint8_t i = 0; i < n; i++) { Serial.print(' '); Serial.print(buf[i]); }
    Serial.println();
  }
  st.ok = true;
  return true;
}

static void printStats(const char* label, const BlockStats &st) {
  Serial.printf("  %s: mean=%.1f  min=%ld  max=%ld  peak-to-peak=%ld  stddev=%.1f counts\n",
                label, st.mean, st.minRaw, st.maxRaw, st.pp, st.stddev);
}

static void printNoDataHelp() {
  Serial.println("  ERROR: HX711 did not become ready (DT stayed HIGH) within the timeout.");
  Serial.println("  This is a COMMUNICATION problem. It does NOT prove the load cell is faulty,");
  Serial.println("  and it is NOT caused by the calibration factor. Type D for a full diagnostic.");
}

static void printChannelHeader(uint8_t ch) {
  Serial.printf("[%s]  DT=GPIO%u  SCK=GPIO%u\n", CH_NAME[ch], DT_PIN[ch], SCK_PIN[ch]);
}

// ============================== COMMANDS ====================================

static void printHelp() {
  Serial.println();
  Serial.println("===== COMMANDS (type one letter, then Enter if your monitor needs it) =====");
  Serial.println("  1 / 2 / 3 : select Plastic / Food / Metal");
  Serial.println("  A         : quick data-ready check of ALL three HX711 modules");
  Serial.println("  D         : full diagnostic of the selected channel");
  Serial.println("  T         : tare (zero) - load cell EMPTY and still");
  Serial.println("  C         : start calibration with the known weight, then G to measure");
  Serial.println("  G         : (after C) weight is placed and still -> measure now");
  Serial.println("  R         : stream raw readings");
  Serial.println("  W         : stream weight in grams (needs a successful calibration)");
  Serial.println("  X         : stop streaming / cancel calibration");
  Serial.println("  F         : print summary of verified factors (for copying)");
  Serial.println("  H         : this help");
  Serial.println("===========================================================================");
}

static void selectChannel(uint8_t ch) {
  active = ch;
  mode = MODE_IDLE;
  Serial.printf("\nSelected channel: %s (DT=GPIO%u, SCK=GPIO%u). Tare: %s | Calibrated: %s\n",
                CH_NAME[ch], DT_PIN[ch], SCK_PIN[ch],
                tareDone[ch] ? "yes" : "no", calDone[ch] ? "yes" : "no");
}

static void quickCheckAll() {
  Serial.println("\n--- Quick data-ready check, all channels ---");
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    long r = 0;
    uint8_t ok = 0;
    long mn = 0, mx = 0;
    for (uint8_t i = 0; i < 3; i++) {
      if (readSample(ch, r)) {
        if (ok == 0) { mn = r; mx = r; }
        if (r < mn) mn = r;
        if (r > mx) mx = r;
        ok++;
      }
    }
    if (ok == 3) Serial.printf("  %-8s DT=GPIO%-2u SCK=GPIO%-2u : OK  (3/3 samples, last raw=%ld, spread=%ld)\n",
                               CH_NAME[ch], DT_PIN[ch], SCK_PIN[ch], r, mx - mn);
    else         Serial.printf("  %-8s DT=GPIO%-2u SCK=GPIO%-2u : NO DATA (%u/3 samples) -> run D on this channel\n",
                               CH_NAME[ch], DT_PIN[ch], SCK_PIN[ch], ok);
  }
}

static void runDiagnostic(uint8_t ch) {
  Serial.println("\n================ DIAGNOSTIC ================");
  printChannelHeader(ch);
  if (DT_PIN[ch] == 36)
    Serial.println("  Note: GPIO36 is input-only on the ESP32 (fine for DT). It has no internal pull-up.");

  // Make sure SCK is LOW: SCK held HIGH for >60 us puts the HX711 into power-down (DT stays HIGH).
  hx[ch].power_up();
  delay(500);

  // Phase 1: watch DT without clocking the chip
  uint16_t total = 0, lows = 0, falls = 0;
  int prev = digitalRead(DT_PIN[ch]);
  uint32_t t0 = millis();
  while (millis() - t0 < 1200) {
    int v = digitalRead(DT_PIN[ch]);
    if (v == LOW) lows++;
    if (prev == HIGH && v == LOW) falls++;
    prev = v;
    total++;
    delay(1);
  }
  Serial.printf("  Phase 1 (DT level for 1.2 s, no clocking): LOW %u of %u checks, %u falling edges\n", lows, total, falls);
  if (lows == 0)
    Serial.println("    -> DT stayed HIGH the whole time. The HX711 is not signalling 'data ready'.");
  else if (lows == total)
    Serial.println("    -> DT stayed LOW. Usually means 'data ready, waiting to be read' (normal). Phase 2 will confirm.");
  else
    Serial.println("    -> DT changed level without being read. Unusual: check Phase 2, wiring, noise.");

  // Phase 2: read 10 samples, time them
  const uint8_t N = 10;
  long raws[N];
  uint8_t got = 0, timeouts = 0, highAfterRead = 0, timedN = 0;
  uint32_t wMin = 0xFFFFFFFFUL, wMax = 0, wSum = 0;
  uint32_t tPrev = micros();
  for (uint8_t i = 0; i < N; i++) {
    uint32_t t1 = micros();
    bool ok = false;
    while ((uint32_t)(micros() - t1) < READY_TIMEOUT_MS * 1000UL) {
      if (hx[ch].is_ready()) { ok = true; break; }
      delayMicroseconds(200);
    }
    if (!ok) { timeouts++; tPrev = micros(); continue; }
    uint32_t waited = micros() - tPrev;       // time since previous read finished
    long r = hx[ch].read();
    if (digitalRead(DT_PIN[ch]) == HIGH) highAfterRead++;  // DT should jump HIGH right after a read
    tPrev = micros();
    raws[got++] = r;
    if (i >= 1 && got >= 2) {                 // first interval is not meaningful
      timedN++;
      wSum += waited;
      if (waited < wMin) wMin = waited;
      if (waited > wMax) wMax = waited;
    }
  }
  Serial.printf("  Phase 2 (10 timed reads): %u ok, %u timeouts (timeout = %lu ms each)\n",
                got, timeouts, (unsigned long)READY_TIMEOUT_MS);

  if (got == 0) {
    Serial.println("  RESULT: NO DATA. DT never went LOW. Check in this order (most to least likely):");
    Serial.printf ("    1. Wiring matches code: DT -> GPIO%u, SCK -> GPIO%u (not swapped, not a neighbouring pin).\n", DT_PIN[ch], SCK_PIN[ch]);
    Serial.println("    2. HX711 VCC at 3.3 V and GND connected to ESP32 GND (measure with a multimeter).");
    Serial.println("    3. Loose jumper / breadboard contact / dry solder joint on VCC, GND, DT or SCK.");
    Serial.println("    4. SCK shorted to 3.3 V (holds chip in power-down) or DT shorted to 3.3 V.");
    Serial.println("    5. Faulty HX711 module (swap with another module to test).");
    Serial.println("  This is not evidence of a bad load cell.");
    Serial.println("============================================");
    return;
  }

  long mn = raws[0], mx = raws[0];
  uint8_t zeros = 0, minusOnes = 0, saturated = 0;
  for (uint8_t i = 0; i < got; i++) {
    if (raws[i] < mn) mn = raws[i];
    if (raws[i] > mx) mx = raws[i];
    if (raws[i] == 0) zeros++;
    if (raws[i] == -1) minusOnes++;
    if (raws[i] >= 8388607L || raws[i] <= -8388608L) saturated++;
  }
  Serial.print("  raw:");
  for (uint8_t i = 0; i < got; i++) { Serial.print(' '); Serial.print(raws[i]); }
  Serial.println();
  Serial.printf("  raw min=%ld max=%ld peak-to-peak=%ld\n", mn, mx, mx - mn);

  if (timedN > 0) {
    float avgMs = (wSum / (float)timedN) / 1000.0f;
    Serial.printf("  Time from end of one read to next data-ready: min=%.1f avg=%.1f max=%.1f ms (about %.1f samples/s)\n",
                  wMin / 1000.0f, avgMs, wMax / 1000.0f, 1000.0f / avgMs);
    if (avgMs > 70 && avgMs < 140)       Serial.println("    -> ~10 samples/s: NORMAL for HX711 with RATE pin LOW (most modules).");
    else if (avgMs > 9 && avgMs < 18)    Serial.println("    -> ~80 samples/s: NORMAL for HX711 with RATE pin HIGH.");
    else                                 Serial.println("    -> Timing is outside the normal 10 / 80 samples/s. Suspect power, noise or a loose contact.");
  }
  Serial.printf("  DT went HIGH immediately after each read: %u of %u (expected: all)\n", highAfterRead, got);
  if (highAfterRead < got)
    Serial.println("    -> DT stayed LOW after a read: possible DT short to GND or wrong pin.");

  if (timeouts > 0)
    Serial.println("  WARNING: some reads timed out -> INTERMITTENT communication (loose contact, power dip, noise).");
  if (zeros == got)
    Serial.println("  WARNING: every raw value is exactly 0 -> DT may be stuck LOW or shorted to GND.");
  if (minusOnes == got)
    Serial.println("  WARNING: every raw value is -1 (all bits 1) -> DT may be stuck HIGH while clocking, or chip absent.");
  if (saturated > 0)
    Serial.println("  WARNING: ADC at its limit (+-8388607). HX711 IS communicating, but its analog input is out of range:\n"
                   "           check load cell wires on E+/E-/A+/A- (open or swapped signal wire, shorted cable).");
  if (got >= 2 && (mx - mn) > 20000)
    Serial.println("  WARNING: large raw fluctuation. Load cell may be moving, cable vibrating, or electrical noise present.");

  if (timeouts == 0 && zeros != got && minusOnes != got && saturated == 0 && highAfterRead == got)
    Serial.println("  RESULT: HX711 communication looks HEALTHY (data-ready timing and data are plausible).");
  else
    Serial.println("  RESULT: see warnings above. They are clues, not proof; change one thing at a time and re-run D.");
  Serial.println("  Note: this test cannot judge the load cell itself. Use T and C to check its response.");
  Serial.println("============================================");
}

static void runTare(uint8_t ch) {
  Serial.println("\n--- TARE (zero) ---");
  printChannelHeader(ch);
  Serial.println("  Make sure the platform is EMPTY and nothing is touching or moving it.");
  BlockStats st;
  if (!measureBlock(ch, SETTLE_DISCARD, SAMPLES_PER_BLOCK, st, true)) {
    tareDone[ch] = false;
    Serial.printf("  TARE FAILED: only %u good samples, %u timeouts.\n", st.good, st.missed);
    printNoDataHelp();
    return;
  }
  printStats("unloaded", st);
  if (st.minRaw <= -RAW_NEAR_LIMIT || st.maxRaw >= RAW_NEAR_LIMIT) {
    tareDone[ch] = false;
    Serial.println("  TARE REJECTED: ADC is at its limit. Check load cell wiring (see D).");
    return;
  }
  if (st.pp > TARE_MAX_PP_COUNTS) {
    tareDone[ch] = false;
    Serial.printf("  TARE REJECTED: readings fluctuate too much (peak-to-peak %ld > %ld counts).\n", st.pp, TARE_MAX_PP_COUNTS);
    Serial.println("  Remove vibration, keep hands/cables away, wait 1-2 min after power-up, then press T again.");
    return;
  }
  long off = lroundf((float)st.mean);
  hx[ch].set_offset(off);
  offsetVal[ch] = off;
  tarePP[ch] = st.pp;
  tareDone[ch] = true;
  Serial.printf("  TARE OK. OFFSET = %ld counts (stored in the HX711 object for this session).\n", off);
  Serial.println("  Next: type C to calibrate, or W (if already calibrated) to see grams.");
}

static void startCalibration(uint8_t ch) {
  Serial.println("\n--- CALIBRATION ---");
  printChannelHeader(ch);
  if (!tareDone[ch]) {
    Serial.println("  Do the tare first (T) with the platform empty. A calibration needs a valid zero.");
    return;
  }
  Serial.printf("  1. Place the %.0f g weight in the CENTER of this compartment's platform.\n", REF_WEIGHT_G);
  Serial.println("  2. Do not touch it. Wait about 10 seconds for it to settle.");
  Serial.println("  3. Type G to measure. Type X to cancel.");
  mode = MODE_WAIT_WEIGHT;
}

static void runCalibration(uint8_t ch) {
  Serial.println("\n--- MEASURING KNOWN WEIGHT ---");
  printChannelHeader(ch);
  BlockStats a, b;
  bool okA = measureBlock(ch, SETTLE_DISCARD + 2, SAMPLES_PER_BLOCK, a, true);
  if (!okA) {
    Serial.printf("  CALIBRATION ABORTED (block A): %u good samples, %u timeouts.\n", a.good, a.missed);
    printNoDataHelp();
    mode = MODE_IDLE;
    return;
  }
  printStats("block A", a);
  bool okB = measureBlock(ch, 0, SAMPLES_PER_BLOCK, b, true);
  if (!okB) {
    Serial.printf("  CALIBRATION ABORTED (block B): %u good samples, %u timeouts.\n", b.good, b.missed);
    printNoDataHelp();
    mode = MODE_IDLE;
    return;
  }
  printStats("block B", b);

  long   offset = hx[ch].get_offset();
  double mean   = (a.mean + b.mean) / 2.0;
  double delta  = mean - (double)offset;      // counts caused by the known weight
  double absD   = fabs(delta);
  Serial.printf("  OFFSET (tare) = %ld | loaded mean = %.1f | delta = %.1f counts\n", offset, mean, delta);

  bool reject = false;
  if (a.minRaw <= -RAW_NEAR_LIMIT || a.maxRaw >= RAW_NEAR_LIMIT || b.minRaw <= -RAW_NEAR_LIMIT || b.maxRaw >= RAW_NEAR_LIMIT) {
    Serial.println("  REJECT: ADC is at its limit (saturated). Check load cell wiring.");
    reject = true;
  }
  if (absD < (double)MIN_DELTA_COUNTS) {
    Serial.printf("  REJECT: response (%.1f counts) is too small. Is the %.0f g weight really on THIS load cell,\n"
                  "          is the cell mounted/wired correctly, and was the tare done on the same channel?\n", absD, REF_WEIGHT_G);
    reject = true;
  }
  double limitPP = (double)MAX_PP_FRACTION * absD;
  if ((double)a.pp > limitPP || (double)b.pp > limitPP) {
    Serial.printf("  REJECT: loaded readings too noisy (pp A=%ld, B=%ld; allowed %.0f). Weight moving or vibration?\n", a.pp, b.pp, limitPP);
    reject = true;
  }
  if ((double)tarePP[ch] > limitPP) {
    Serial.printf("  REJECT: the tare was too noisy for this signal size (tare pp=%ld; allowed %.0f). Repeat T in a quieter setup.\n", tarePP[ch], limitPP);
    reject = true;
  }
  if (fabs(a.mean - b.mean) > (double)MAX_MISMATCH_FRACTION * absD) {
    Serial.printf("  REJECT: block A and B differ by %.1f counts (allowed %.1f): reading is drifting or the weight is still settling.\n",
                  fabs(a.mean - b.mean), (double)MAX_MISMATCH_FRACTION * absD);
    reject = true;
  }
  if (reject) {
    Serial.println("  CALIBRATION REJECTED. No factor was produced and the previous factor (if any) is unchanged.");
    Serial.println("  Fix the problem shown above, then type C again.");
    mode = MODE_IDLE;
    return;
  }

  float factor = (float)(delta / (double)REF_WEIGHT_G);   // counts per gram
  hx[ch].set_scale(factor);
  scaleVal[ch]  = factor;
  offsetVal[ch] = offset;
  calDone[ch]   = true;

  double sd = (a.stddev > b.stddev) ? a.stddev : b.stddev;
  Serial.println("  CALIBRATION ACCEPTED");
  Serial.printf("  CALIBRATION FACTOR (%s) = %.4f counts per gram\n", CH_NAME[ch], factor);
  Serial.printf("  Code line: scale_%s.set_scale(%.4ff);\n", CH_NAME[ch], factor);
  if (factor < 0)
    Serial.println("  Factor is NEGATIVE: raw value decreases when weight is added (wiring orientation). This is valid; keep the minus sign.");
  Serial.printf("  Noise estimate: stddev ~ %.2f g, peak-to-peak ~ %.2f g\n", sd / fabs(factor), (double)((a.pp > b.pp) ? a.pp : b.pp) / fabs(factor));
  Serial.println("  Live grams follow. Remove the weight -> expect about 0 g. Put it back -> expect about 250 g. X to stop.");

  mode = MODE_IDLE;
  // start live weight view
  ringCount = 0; ringIdx = 0;
  lastSampleMs = millis(); lastPrintMs = lastSampleMs;
  mode = MODE_STREAM_WEIGHT;
}

static void startStream(Mode m) {
  if (m == MODE_STREAM_WEIGHT && !calDone[active]) {
    Serial.println("\nThis channel is not calibrated yet. Use R for raw values, or T then C to calibrate.");
    return;
  }
  ringCount = 0; ringIdx = 0;
  lastSampleMs = millis(); lastPrintMs = lastSampleMs;
  mode = m;
  Serial.printf("\nStreaming %s for %s. Type X to stop.\n", m == MODE_STREAM_RAW ? "RAW" : "WEIGHT", CH_NAME[active]);
  if (m == MODE_STREAM_WEIGHT)
    Serial.println("Negative grams are shown as they are (not clamped). They mean the force is opposite to the calibration direction.");
}

static void printSummary() {
  Serial.println("\n===== VERIFIED FACTORS (this session) =====");
  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    if (calDone[ch]) Serial.printf("  %-8s : %.4f counts/gram   ->  set_scale(%.4ff);\n", CH_NAME[ch], scaleVal[ch], scaleVal[ch]);
    else             Serial.printf("  %-8s : NOT calibrated yet\n", CH_NAME[ch]);
  }
  Serial.println("  Factors are lost on reset/power-off. Copy them into your notes now.");
  Serial.println("  (OFFSET/tare is NOT copied: the main firmware must tare on its own at startup.)");
  Serial.println("===========================================");
}

// ============================== LIVE STREAM =================================

static void streamTick() {
  uint8_t ch = active;
  uint32_t now = millis();
  if (hx[ch].is_ready()) {
    long raw = hx[ch].read();            // ready confirmed above, so this does not block
    lastRaw = raw;
    lastSampleMs = now;
    ring[ringIdx] = raw;
    ringIdx = (ringIdx + 1) % RING_SIZE;
    if (ringCount < RING_SIZE) ringCount++;
  }
  if (now - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = now;
  if (ringCount == 0 || now - lastSampleMs > STALL_WARN_MS) {
    Serial.printf("WARNING: no new HX711 data for %lu ms (communication problem, not a calibration problem). Type D.\n",
                  (unsigned long)(now - lastSampleMs));
    return;
  }
  double sum = 0;
  for (uint8_t i = 0; i < ringCount; i++) sum += (double)ring[i];
  double avg = sum / ringCount;
  double net = avg - (double)hx[ch].get_offset();
  if (mode == MODE_STREAM_RAW) {
    Serial.printf("%s | raw=%ld | raw avg(%u)=%.1f | net(raw-offset)=%.1f counts\n", CH_NAME[ch], lastRaw, ringCount, avg, net);
  } else {
    double scale = (double)hx[ch].get_scale();           // same formula as get_units()
    double g     = net / scale;
    double gNow  = ((double)lastRaw - (double)hx[ch].get_offset()) / scale;
    Serial.printf("%s | raw avg(%u)=%.1f | net=%.1f counts | weight=%.2f g (latest sample %.2f g)\n",
                  CH_NAME[ch], ringCount, avg, net, g, gNow);
  }
}

// ============================== SERIAL INPUT ================================

static void handleCommand(char c) {
  if (c == '\r' || c == '\n' || c == ' ') return;
  switch (tolower(c)) {
    case '1': selectChannel(0); break;
    case '2': selectChannel(1); break;
    case '3': selectChannel(2); break;
    case 'h': case '?': printHelp(); break;
    case 'a': mode = MODE_IDLE; quickCheckAll(); break;
    case 'd': mode = MODE_IDLE; runDiagnostic(active); break;
    case 't': mode = MODE_IDLE; runTare(active); break;
    case 'c': startCalibration(active); break;
    case 'g':
      if (mode == MODE_WAIT_WEIGHT) runCalibration(active);
      else Serial.println("\nNothing to measure. Type C first.");
      break;
    case 'r': startStream(MODE_STREAM_RAW); break;
    case 'w': startStream(MODE_STREAM_WEIGHT); break;
    case 'x':
      if (mode != MODE_IDLE) Serial.println("\nStopped.");
      mode = MODE_IDLE;
      break;
    case 'f': printSummary(); break;
    default: Serial.printf("\nUnknown command '%c'. Type H for help.\n", c); break;
  }
}

// ============================== ARDUINO =====================================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("SmartWaste Pro - HX711 calibration tool (standalone)");
  Serial.printf("Reference weight: %.0f g | Library: bogde/HX711\n", REF_WEIGHT_G);

  for (uint8_t ch = 0; ch < NUM_CH; ch++) {
    hx[ch].begin(DT_PIN[ch], SCK_PIN[ch], 128);  // channel A, gain 128
    hx[ch].set_scale(1.0f);
    hx[ch].set_offset(0);
    Serial.printf("  %-8s DT=GPIO%-2u SCK=GPIO%-2u\n", CH_NAME[ch], DT_PIN[ch], SCK_PIN[ch]);
  }
  Serial.println("Allow 1-2 minutes of warm-up before taring for best accuracy.");
  printHelp();
  selectChannel(0);
}

void loop() {
  while (Serial.available() > 0) handleCommand((char)Serial.read());
  if (mode == MODE_STREAM_RAW || mode == MODE_STREAM_WEIGHT) streamTick();
  delay(1);   // yields to the ESP32 background tasks; no long blocking waits in loop()
}