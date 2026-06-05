// ============================================================
//  8-Wire Cable Tester v4.0 — Arduino Nano + I2C 16x2 LCD
//
//  WIRING:
//    No external resistors needed — uses INPUT_PULLUP (active-low)
//    TX pins : D2, D3, D4, D5, D6, D7, D8, D9
//    RX pins : A0, A1, A2, A3, D10, D0, D11, D12
//    LCD SDA : A4
//    LCD SCL : A5
//
//  NOTE: Serial is disabled — D0 is used as RX6.
//        D13 is not used as a test pin (onboard LED causes false reads).
//
//  LCD SEQUENCE PER RUN:
//    Screen 1 : Progress during testing
//    Screen 2 : Summary grid  "1 2 3 4 5 6 7 8 / P P P F P W P P"
//    Screen 3 : Overall result + avg propagation delay
//    Screens 4-19 : Per wire, two screens each —
//      A) Wire N: PASS/WARN/FAIL  +  primary fault or voltage
//      B) Contact resistance (Ω) or error counts + delay + bleed
//    Screen 20: "Retest in 3s..."
//
//  ACTIVE-LOW LOGIC:
//    Drive TX LOW  = send signal
//    Release TX    = INPUT_PULLUP = idle HIGH
//    Open wire     = RX stays HIGH when TX driven LOW
//    Short         = RX goes LOW when a different TX is driven LOW
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // change to 0x3F if LCD is blank

// ── Pin assignments ───────────────────────────────────────────
const int TX_PINS[8]       = {2, 3, 4, 5, 6, 7, 8, 9};
const int RX_PINS[8]       = {A0, A1, A2, A3, 10, 0, 11, 12};
const bool RX_HAS_ANALOG[8]= {true, true, true, true, false, false, false, false};
// A0-A3 support analogRead(); D0, D10, D11, D12 do not.

// ── Tuning constants ──────────────────────────────────────────
const int   SETTLE_US        = 500;   // µs after driving — increase for long cables
const int   TEST_REPEAT      = 3;     // PRBS pattern repetitions per phase
const int   PAUSE_MS         = 2500;  // ms each LCD screen is displayed
const int   HISTORY_DEPTH    = 10;    // runs kept for intermittent detection
const int   SHORT_THRESHOLD  = 5;     // shortCount hits needed to flag a short
const int   DECAY_CLEAN_READS= 3;     // consecutive clean reads to decay shortCount
const int   BLEED_WARN_MIN   = 10;    // minimum Walsh bleed count to trigger WARN

const float PULLUP_OHMS      = 50000.0; // Nano internal pull-up ~50kΩ
const float VCC              = 5.0;

// Voltage WARN thresholds
const float WARN_V_HIGH_MIN  = VCC * 0.70; // idle voltage must be above this
const float WARN_V_LOW_MAX   = VCC * 0.30; // active voltage must be below this
const float WARN_R_MAX       = 2000.0;     // contact resistance WARN threshold (Ω)
const float WARN_DELAY_MAX   = 1000.0;     // propagation delay WARN threshold (µs)

// ── PRBS test patterns (active-low: 1 = drive LOW, 0 = release) ──
const uint32_t PRBS_A = 0b10110100110010101111000001101001;
const uint32_t PRBS_B = 0b01001011001101010000111110010110;

// ── Walsh-Hadamard H8 matrix ──────────────────────────────────
const uint8_t WALSH[8] = {
  0b11111111,
  0b11110000,
  0b11001100,
  0b10101010,
  0b11000011,
  0b10100101,
  0b10010110,
  0b10000001
};

// ── Severity levels ───────────────────────────────────────────
enum Severity { SEV_PASS, SEV_WARN, SEV_FAIL };

// ── Per-wire result struct ────────────────────────────────────
struct WireResult {
  Severity severity;
  bool openFault;          // no signal in forward direction
  bool openFaultReverse;   // no signal in reverse only (asymmetric)
  bool idleBleed;          // drive pin reads LOW when it should be idle
  bool shortWith[8];       // confirmed short with each other wire
  bool crossTo;            // signal appeared on a different wire (mis-wire)
  int  crossTarget;        // which wire the signal appeared on
  int  seqBitErrors;       // sequential forward bit errors
  int  seqBitErrorsRev;    // sequential reverse bit errors
  int  parBitErrors;       // Walsh self errors (diagonal)
  int  totalBleed;         // Walsh off-diagonal bleed count
  bool analogValid;        // true if this RX pin supports analogRead
  float avgVoltageActive;  // avg voltage when TX driven LOW (ideal: 0V)
  float avgVoltageIdle;    // avg voltage when TX released HIGH (ideal: 5V)
  float estimatedResistance; // contact+wire resistance in Ω
  float propagationDelayUs;  // signal propagation delay in µs
  bool  intermittent;      // failed some runs but not all
  int   passCount;         // number of passing runs in history
};

WireResult results[8];
int  faultMatrix[8][8];
bool passHistory[8][HISTORY_DEPTH];
int  historyIndex = 0;
int  totalRuns    = 0;

// ── Forward declarations ──────────────────────────────────────
void runSequentialTest(bool reverse);
void runWalshTest();
void analyzeFaultMatrix();
void computeSeverity();
void updateHistory();
void displayResultsLCD();
void clearResults();
void clearFaultMatrix();
void lcdPrint(const char* line1, const char* line2);
inline void drivePin(int pin);
inline void releasePin(int pin);
float measurePropDelay(int txPin, int rxPin);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  lcd.init();
  lcd.backlight();
  lcdPrint("Cable Tester v4", "Initializing...");

  for (int i = 0; i < 8; i++) {
    pinMode(TX_PINS[i], INPUT_PULLUP);
    pinMode(RX_PINS[i], INPUT_PULLUP);
  }

  for (int w = 0; w < 8; w++)
    for (int r = 0; r < HISTORY_DEPTH; r++)
      passHistory[w][r] = false;

  delay(1000);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  lcdPrint("Cable Tester v4", "Testing...");
  delay(400);

  clearResults();
  clearFaultMatrix();

  lcdPrint("Phase 1a/3", "Seq. forward...");
  runSequentialTest(false);

  lcdPrint("Phase 1b/3", "Seq. reverse...");
  runSequentialTest(true);

  lcdPrint("Phase 2/3", "Walsh parallel..");
  runWalshTest();

  analyzeFaultMatrix();
  computeSeverity();
  updateHistory();
  displayResultsLCD();
}

// ============================================================
//  DRIVE / RELEASE helpers
//  drivePin  — sets OUTPUT LOW  (sends signal, active-low)
//  releasePin — sets INPUT_PULLUP (idle, floats HIGH via 50kΩ)
//  Never drive HIGH as OUTPUT — two outputs fighting causes damage.
// ============================================================
inline void drivePin(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

inline void releasePin(int pin) {
  pinMode(pin, INPUT_PULLUP);
}

// ============================================================
//  PROPAGATION DELAY MEASUREMENT
//  Drives txPin LOW, polls rxPin until it follows.
//  Returns elapsed µs, or -1.0 if no response within 5ms.
// ============================================================
float measurePropDelay(int txPin, int rxPin) {
  releasePin(txPin);
  delayMicroseconds(400);

  unsigned long t0       = micros();
  unsigned long deadline = t0 + 5000UL;
  drivePin(txPin);

  while (micros() < deadline) {
    if (digitalRead(rxPin) == LOW) {
      float d = (float)(micros() - t0);
      releasePin(txPin);
      delayMicroseconds(SETTLE_US * 2);
      return d;
    }
  }
  releasePin(txPin);
  delayMicroseconds(SETTLE_US * 2);
  return -1.0;
}

// ============================================================
//  PHASE 1 — SEQUENTIAL SINGLE-WIRE TEST
//
//  Tests one wire at a time, in both directions.
//  Sends PRBS_A and PRBS_B patterns, checks the expected RX pin
//  and watches idle pins for unexpected LOWs (shorts).
//
//  Short detection uses a hysteresis counter per idle pin:
//  shortCount increments on each unexpected LOW, but requires
//  DECAY_CLEAN_READS consecutive clean reads to decrement by 1.
//  A short is only committed if shortCount >= SHORT_THRESHOLD.
// ============================================================
void runSequentialTest(bool reverse) {
  const int* drivePins = reverse ? RX_PINS : TX_PINS;
  const int* readPins  = reverse ? TX_PINS : RX_PINS;

  for (int i = 0; i < 8; i++) {
    releasePin(drivePins[i]);
    releasePin(readPins[i]);
  }
  delayMicroseconds(SETTLE_US * 4);

  for (int w = 0; w < 8; w++) {

    // Progress bar on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(reverse ? "Rev wire " : "Fwd wire ");
    lcd.print(w + 1);
    lcd.setCursor(0, 1);
    lcd.print("[");
    for (int b = 0; b < w * 2; b++) lcd.print("=");
    lcd.print(">");

    int  bitErrors     = 0;
    long analogSumAct  = 0;
    long analogSumIdle = 0;
    int  analogCntAct  = 0;
    int  analogCntIdle = 0;
    bool localOpen     = false;
    bool localIdle     = false;
    int  shortCount[8] = {};
    int  cleanCount[8] = {};

    // Measure propagation delay on forward pass only
    float propDelay = -1.0;
    if (!reverse)
      propDelay = measurePropDelay(drivePins[w], readPins[w]);

    for (int rep = 0; rep < TEST_REPEAT; rep++) {
      for (int pass = 0; pass < 2; pass++) {
        uint32_t pattern = (pass == 0) ? PRBS_A : PRBS_B;

        for (int bit = 31; bit >= 0; bit--) {
          bool sendLow = (pattern >> bit) & 1;

          if (sendLow) drivePin(drivePins[w]);
          else         releasePin(drivePins[w]);
          delayMicroseconds(SETTLE_US);

          // Read all RX pins at once
          bool rxState[8];
          for (int r = 0; r < 8; r++)
            rxState[r] = (digitalRead(readPins[r]) == LOW);

          // Check expected pin
          if (rxState[w] != sendLow) {
            bitErrors++;
            if (sendLow && !rxState[w]) localOpen = true;
          }

          // Analog sampling on forward pass for analog-capable pins
          if (!reverse && RX_HAS_ANALOG[w]) {
            if (sendLow) { analogSumAct  += analogRead(readPins[w]); analogCntAct++;  }
            else         { analogSumIdle += analogRead(readPins[w]); analogCntIdle++; }
          }

          // Idle pin monitoring
          for (int r = 0; r < 8; r++) {
            if (r == w) continue;

            // Idle drive pin should never read LOW
            if (digitalRead(drivePins[r]) == LOW) localIdle = true;

            // Unexpected LOW on idle RX while we are driving LOW = possible short
            if (sendLow && rxState[r]) {
              shortCount[r]++;
              cleanCount[r] = 0;
            } else {
              cleanCount[r]++;
              if (cleanCount[r] >= DECAY_CLEAN_READS) {
                if (shortCount[r] > 0) shortCount[r]--;
                cleanCount[r] = 0;
              }
            }
          }

        } // bit
      } // pass
    } // repeat

    releasePin(drivePins[w]);
    delayMicroseconds(SETTLE_US * 2);

    // Commit shorts (forward pass only — Walsh confirms later)
    if (!reverse) {
      for (int r = 0; r < 8; r++)
        if (shortCount[r] >= SHORT_THRESHOLD)
          results[w].shortWith[r] = true;
    }

    // Store forward results
    if (!reverse) {
      results[w].seqBitErrors       = bitErrors;
      results[w].openFault          = localOpen;
      results[w].idleBleed          = localIdle;
      results[w].propagationDelayUs = propDelay;
      results[w].analogValid        = RX_HAS_ANALOG[w];

      if (RX_HAS_ANALOG[w] && analogCntAct > 0 && analogCntIdle > 0) {
        results[w].avgVoltageActive = (analogSumAct  / (float)analogCntAct)  * (VCC / 1023.0);
        results[w].avgVoltageIdle   = (analogSumIdle / (float)analogCntIdle) * (VCC / 1023.0);
        // Contact resistance: R = R_pullup * V_active / (Vcc - V_active)
        float va = results[w].avgVoltageActive;
        results[w].estimatedResistance = (va < (VCC - 0.1))
          ? PULLUP_OHMS * va / (VCC - va)
          : -1.0;
      } else {
        results[w].avgVoltageActive    = -1.0;
        results[w].avgVoltageIdle      = -1.0;
        results[w].estimatedResistance = -1.0;
      }

    // Store reverse results
    } else {
      results[w].seqBitErrorsRev  = bitErrors;
      // Asymmetric open: forward clean but reverse fails = diode or one-way contact
      results[w].openFaultReverse = (!results[w].openFault && bitErrors > 0);
    }
  }

  for (int i = 0; i < 8; i++) {
    releasePin(TX_PINS[i]);
    releasePin(RX_PINS[i]);
  }
  delayMicroseconds(SETTLE_US * 4);
}

// ============================================================
//  PHASE 2 — WALSH-HADAMARD PARALLEL TEST
//
//  Drives multiple TX pins simultaneously using the H8 Walsh
//  matrix so every possible combination of wires is exercised.
//  Each phase uses a different Walsh row as the drive pattern.
//
//  Fault matrix logic:
//    Diagonal   (tx==rx): self-error if RX doesn't follow its TX
//    Off-diagonal       : bleed if RX is LOW but its own TX is NOT active
//    Idle-idle          : NOT a fault (both floating HIGH is correct)
// ============================================================
void runWalshTest() {
  uint32_t patterns[2] = {PRBS_A, PRBS_B};

  for (int i = 0; i < 8; i++) releasePin(TX_PINS[i]);
  delayMicroseconds(SETTLE_US * 4);

  for (int phase = 0; phase < 8; phase++) {

    // Progress bar on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Walsh ");
    lcd.print(phase + 1);
    lcd.print("/8 x");
    lcd.print(TEST_REPEAT);
    lcd.setCursor(0, 1);
    lcd.print("[");
    for (int b = 0; b < phase * 2; b++) lcd.print("=");
    lcd.print(">");

    for (int rep = 0; rep < TEST_REPEAT; rep++) {
      for (int p = 0; p < 2; p++) {
        for (int bit = 31; bit >= 0; bit--) {
          bool bitActive = (patterns[p] >> bit) & 1;

          bool txActive[8];
          for (int tx = 0; tx < 8; tx++) {
            txActive[tx] = bitActive && ((WALSH[tx] >> (7 - phase)) & 1);
            if (txActive[tx]) drivePin(TX_PINS[tx]);
            else              releasePin(TX_PINS[tx]);
          }
          delayMicroseconds(SETTLE_US);

          bool rxState[8];
          for (int rx = 0; rx < 8; rx++)
            rxState[rx] = (digitalRead(RX_PINS[rx]) == LOW);

          for (int rx = 0; rx < 8; rx++) {
            // Diagonal: self-error if RX doesn't match its TX state
            if (rxState[rx] != txActive[rx])
              faultMatrix[rx][rx]++;

            // Off-diagonal: RX LOW when its own TX is NOT active = bleed
            if (rxState[rx] && !txActive[rx]) {
              for (int tx = 0; tx < 8; tx++) {
                if (tx != rx && txActive[tx])
                  faultMatrix[tx][rx]++;
              }
            }
          }

        } // bit
      } // prbs pass
    } // repeat

    for (int tx = 0; tx < 8; tx++) releasePin(TX_PINS[tx]);
    delayMicroseconds(SETTLE_US * 2);
  }

  for (int i = 0; i < 8; i++) releasePin(TX_PINS[i]);
}

// ============================================================
//  ANALYZE FAULT MATRIX
//
//  Snapshots sequential shorts, clears them, then re-confirms
//  only those backed by Walsh evidence too. This prevents
//  capacitive coupling (which only appears in Walsh) from being
//  promoted to a hard short, and prevents sequential noise from
//  surviving without Walsh confirmation.
//
//  A hard SHORT requires all three:
//    1. Sequential phase saw it bidirectionally
//    2. Walsh bleed is bidirectional
//    3. Walsh bleed count > 10% of total bits
// ============================================================
void analyzeFaultMatrix() {
  int totalBits = 8 * 2 * 32 * TEST_REPEAT;

  // Snapshot then clear sequential shorts
  bool seqShorts[8][8];
  for (int w = 0; w < 8; w++)
    for (int r = 0; r < 8; r++) {
      seqShorts[w][r]         = results[w].shortWith[r];
      results[w].shortWith[r] = false;
    }

  for (int w = 0; w < 8; w++) {
    results[w].parBitErrors = faultMatrix[w][w];
    results[w].totalBleed   = 0;

    for (int r = 0; r < 8; r++) {
      if (r == w) continue;
      if (faultMatrix[w][r] == 0) continue;

      results[w].totalBleed += faultMatrix[w][r];

      // Cross-wire: self is open + another wire has strong signal = mis-wired
      if (results[w].openFault && faultMatrix[w][r] > (totalBits / 4)) {
        results[w].crossTo     = true;
        results[w].crossTarget = r;
        continue;
      }

      // Confirm hard short
      bool seqBidi   = seqShorts[w][r] && seqShorts[r][w];
      bool walshBidi = faultMatrix[r][w] > 0;
      bool walshHeavy= faultMatrix[w][r] > (totalBits / 10);
      if (seqBidi && walshBidi && walshHeavy)
        results[w].shortWith[r] = true;
      // else: bleed only, totalBleed already incremented → may trigger WARN
    }
  }
}

// ============================================================
//  COMPUTE SEVERITY
// ============================================================
void computeSeverity() {
  int seqTotalBits = 2 * 32 * TEST_REPEAT;

  for (int w = 0; w < 8; w++) {
    bool anyShort = false;
    for (int r = 0; r < 8; r++)
      if (results[w].shortWith[r]) anyShort = true;

    bool hardFail =
      results[w].openFault                          ||
      results[w].crossTo                            ||
      anyShort                                      ||
      results[w].seqBitErrors  > (seqTotalBits / 4)||
      results[w].parBitErrors  > 0;

    bool warnFlag =
      results[w].openFaultReverse                   ||
      results[w].intermittent                       ||
      results[w].idleBleed                          ||
      results[w].seqBitErrors    > 0                ||
      results[w].seqBitErrorsRev > 0                ||
      results[w].totalBleed      > BLEED_WARN_MIN   ||
      (results[w].analogValid && results[w].avgVoltageIdle   >= 0 &&
       results[w].avgVoltageIdle   < WARN_V_HIGH_MIN)            ||
      (results[w].analogValid && results[w].avgVoltageActive >= 0 &&
       results[w].avgVoltageActive > WARN_V_LOW_MAX)             ||
      (results[w].analogValid &&
       results[w].estimatedResistance > WARN_R_MAX)              ||
      (results[w].propagationDelayUs  > 0 &&
       results[w].propagationDelayUs  > WARN_DELAY_MAX);

    if      (hardFail) results[w].severity = SEV_FAIL;
    else if (warnFlag) results[w].severity = SEV_WARN;
    else               results[w].severity = SEV_PASS;
  }
}

// ============================================================
//  PASS HISTORY & INTERMITTENT DETECTION
// ============================================================
void updateHistory() {
  totalRuns = min(totalRuns + 1, HISTORY_DEPTH);
  for (int w = 0; w < 8; w++) {
    passHistory[w][historyIndex] = (results[w].severity == SEV_PASS);
    int passes = 0;
    int runs   = min(totalRuns, HISTORY_DEPTH);
    for (int r = 0; r < runs; r++)
      passes += passHistory[w][r] ? 1 : 0;
    results[w].passCount    = passes;
    results[w].intermittent = (passes > 0 && passes < runs);
  }
  historyIndex = (historyIndex + 1) % HISTORY_DEPTH;
}

// ============================================================
//  LCD DISPLAY
//
//  Screen sequence:
//    1. Summary grid     — "1 2 3 4 5 6 7 8" / "P P F P W P P P"
//    2. Overall result   — "ALL 8 PASS :-)" or "2xFAIL 1xWARN"
//                          + avg propagation delay on line 2
//    3-18. Per wire (2 screens each):
//      A. "Wn: PASS/WARN/FAIL [history]" / primary fault or voltage
//      B. "R:790Ω A:0.078V"             / "Dly:20us Bl:0"
//    19. "Retest in 3s..."
// ============================================================
void displayResultsLCD() {

  // ── Screen 1: Summary grid ────────────────────────────────
  lcd.clear();
  lcd.setCursor(0, 0);
  for (int w = 0; w < 8; w++) {
    lcd.print(w + 1);
    lcd.print(" ");
  }
  lcd.setCursor(0, 1);
  for (int w = 0; w < 8; w++) {
    switch (results[w].severity) {
      case SEV_PASS: lcd.print("P"); break;
      case SEV_WARN: lcd.print("W"); break;
      case SEV_FAIL: lcd.print("F"); break;
    }
    lcd.print(" ");
  }
  delay(PAUSE_MS);

  // ── Screen 2: Overall result + avg delay ──────────────────
  lcd.clear();
  int failCount = 0, warnCount = 0;
  for (int i = 0; i < 8; i++) {
    if (results[i].severity == SEV_FAIL) failCount++;
    if (results[i].severity == SEV_WARN) warnCount++;
  }
  lcd.setCursor(0, 0);
  if (failCount == 0 && warnCount == 0) {
    lcd.print("ALL 8 PASS  :-)");
  } else {
    if (failCount > 0) { lcd.print(failCount); lcd.print("xFAIL "); }
    if (warnCount > 0) { lcd.print(warnCount); lcd.print("xWARN"); }
  }
  lcd.setCursor(0, 1);
  float avgDelay = 0; int dCnt = 0;
  for (int w = 0; w < 8; w++)
    if (results[w].propagationDelayUs > 0 && results[w].severity == SEV_PASS) {
      avgDelay += results[w].propagationDelayUs;
      dCnt++;
    }
  if (dCnt > 0) {
    lcd.print("Avg dly:");
    lcd.print((int)(avgDelay / dCnt));
    lcd.print("us");
  }
  delay(PAUSE_MS);

  // ── Screens 3-18: Per wire ────────────────────────────────
  for (int w = 0; w < 8; w++) {

    // ── Screen A: Status + primary fault/reading ──────────
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("W"); lcd.print(w + 1); lcd.print(": ");
    switch (results[w].severity) {
      case SEV_PASS: lcd.print("PASS"); break;
      case SEV_WARN: lcd.print("WARN"); break;
      case SEV_FAIL: lcd.print("FAIL"); break;
    }
    // Pass history top-right e.g. " 9/10"
    if (totalRuns > 1) {
      char hist[7];
      snprintf(hist, sizeof(hist), " %d/%d", results[w].passCount, min(totalRuns, HISTORY_DEPTH));
      int col = 16 - strlen(hist);
      if (col > 7) { lcd.setCursor(col, 0); lcd.print(hist); }
    }

    lcd.setCursor(0, 1);
    if (results[w].openFault) {
      lcd.print(results[w].propagationDelayUs < 0 ? "No signal-check" : "Noisy/bad crimp?");
    } else if (results[w].crossTo) {
      lcd.print("->W"); lcd.print(results[w].crossTarget + 1); lcd.print(" mis-wired");
    } else if (results[w].openFaultReverse) {
      lcd.print("Asym:diode/pin?");
    } else {
      // Check for shorts first
      bool printedShort = false;
      for (int r = 0; r < 8; r++) {
        if (results[w].shortWith[r]) {
          if (!printedShort) { lcd.print("Short->W"); lcd.print(r + 1); printedShort = true; }
          else               { lcd.print("+"); lcd.print(r + 1); }
        }
      }
      if (!printedShort) {
        if (results[w].idleBleed) {
          lcd.print("Board leakage!");
        } else if (results[w].intermittent) {
          lcd.print("Intermittent!");
        } else if (results[w].severity == SEV_PASS) {
          if (results[w].analogValid && results[w].avgVoltageIdle > 0) {
            lcd.print("Idle:");
            lcd.print(results[w].avgVoltageIdle, 2);
            lcd.print("V OK");
          } else {
            lcd.print("Signal OK");
          }
        } else {
          int errs = results[w].seqBitErrors + results[w].seqBitErrorsRev;
          if (errs > 0) { lcd.print("Marginal:"); lcd.print(errs); lcd.print("err"); }
          else            lcd.print("Check signal");
        }
      }
    }
    delay(PAUSE_MS);

    // ── Screen B: Measurements ────────────────────────────
    lcd.clear();
    lcd.setCursor(0, 0);
    if (results[w].analogValid && results[w].estimatedResistance >= 0) {
      // Contact resistance measured via voltage divider against 50kΩ pullup
      // R = 50000 * V_active / (Vcc - V_active)
      lcd.print("R:");
      lcd.print((int)results[w].estimatedResistance);
      lcd.print((char)244);  // Ω character on HD44780
      lcd.print(" A:");
      lcd.print(results[w].avgVoltageActive, 3);
      lcd.print("V");
    } else {
      // Digital-only pin: show bit error counts instead
      lcd.print("Err:");
      lcd.print(results[w].seqBitErrors);
      lcd.print("f/");
      lcd.print(results[w].seqBitErrorsRev);
      lcd.print("r W:");
      lcd.print(results[w].parBitErrors);
    }

    lcd.setCursor(0, 1);
    if (results[w].propagationDelayUs > 0) {
      lcd.print("Dly:");
      lcd.print((int)results[w].propagationDelayUs);
      lcd.print("us");
    } else {
      lcd.print("Dly:N/A");
    }
    lcd.print(" Bl:");
    lcd.print(results[w].totalBleed);

    delay(PAUSE_MS);
  }

  // ── Screen 19: Retest countdown ───────────────────────────
  lcd.clear();
  lcd.setCursor(0, 0);
  if (failCount == 0 && warnCount == 0)
    lcd.print("ALL 8 PASS  :-)");
  else {
    if (failCount > 0) { lcd.print(failCount); lcd.print("xFAIL "); }
    if (warnCount > 0) { lcd.print(warnCount); lcd.print("xWARN"); }
  }
  lcd.setCursor(0, 1);
  lcd.print("Retest in 3s...");
  delay(3000);
}

// ============================================================
//  CLEAR HELPERS
// ============================================================
void clearResults() {
  for (int w = 0; w < 8; w++) {
    results[w].severity            = SEV_PASS;
    results[w].openFault           = false;
    results[w].openFaultReverse    = false;
    results[w].idleBleed           = false;
    results[w].crossTo             = false;
    results[w].crossTarget         = -1;
    results[w].seqBitErrors        = 0;
    results[w].seqBitErrorsRev     = 0;
    results[w].parBitErrors        = 0;
    results[w].totalBleed          = 0;
    results[w].analogValid         = false;
    results[w].avgVoltageActive    = -1.0;
    results[w].avgVoltageIdle      = -1.0;
    results[w].estimatedResistance = -1.0;
    results[w].propagationDelayUs  = -1.0;
    results[w].intermittent        = false;
    results[w].passCount           = 0;
    for (int r = 0; r < 8; r++) results[w].shortWith[r] = false;
  }
}

void clearFaultMatrix() {
  for (int tx = 0; tx < 8; tx++)
    for (int rx = 0; rx < 8; rx++)
      faultMatrix[tx][rx] = 0;
}

void lcdPrint(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}
