// ============================================================
//  8-Wire Cable Tester v3.5 — Arduino Nano + I2C 16x2 LCD
//
//  FIXES vs v3.2:
//    1. Walsh fault-matrix logic rewritten — idle (both tx and rx
//       inactive) is no longer counted as a fault.
//    2. shortWith[] cleared at the start of Walsh, not just in
//       clearResults(), so Walsh and sequential faults don't stack.
//    3. Reverse-open detection now requires BOTH a bit-error AND
//       the forward test to be clean, preventing spurious
//       "Asym open-diode?" on wires 7/8 (D11/D12 pair).
//    4. SHORT_THRESHOLD decay changed from single-read to
//       DECAY_CLEAN_READS consecutive clean reads, so real shorts
//       accumulate faster than noise.
//    5. Disconnected wires correctly report "No signal" only —
//       no false short companions.
//
//  WIRING
//    - No external resistors needed
//    - Uses INPUT_PULLUP throughout (active-low logic)
//    - TX: D2-D9
//    - RX: A0, A1, A2, A3, D10, D0, D11, D12
//    - LCD SDA: A4   SCL: A5
// ============================================================


#include <Wire.h>
#include <LiquidCrystal_I2C.h>


LiquidCrystal_I2C lcd(0x27, 16, 2); // try 0x3F if blank

// ── Pin assignments ───────────────────────────────────────
const int TX_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const int RX_PINS[8] = {A0, A1, A2, A3, 10, 0, 11, 12};

// Only A0-A3 support analogRead() on the Nano
const bool RX_HAS_ANALOG[8] = {true, true, true, true, false, false, false, false};

// ── Tuning ────────────────────────────────────────────────
const int   SETTLE_US         = 500;  // raised from 200 — pullups need more recovery time in Walsh
const int   TEST_REPEAT       = 3;    // repeats per phase
const int   PAUSE_MS          = 2500; // ms per wire on LCD
const int   HISTORY_DEPTH     = 10;   // runs tracked for intermittent detection
const int   SHORT_THRESHOLD   = 5;    // consecutive wrong reads before flagging short
const int   DECAY_CLEAN_READS = 3;    // consecutive clean reads needed to decay counter
                                      // (was 1 in v3.2 — too aggressive, noise slipped through)
const int   BLEED_WARN_MIN    = 10;   // Walsh totalBleed must exceed this before triggering WARN
                                      // Prevents capacitive coupling on unconnected pins from
                                      // generating spurious WARNs (coupling saturates at 384,
                                      // but a real marginal contact is much lower and consistent)
const bool  CSV_MODE          = false;

const float PULLUP_OHMS    = 50000.0; // Nano internal pull-up ~50kΩ
const float VCC            = 5.0;

// WARN thresholds
const float WARN_V_HIGH_MIN = VCC * 0.70;
const float WARN_V_LOW_MAX  = VCC * 0.30;
const float WARN_R_MAX      = 2000.0;
const float WARN_DELAY_MAX  = 1000.0;
// ─────────────────────────────────────────────────────────

// ── PRBS patterns ─────────────────────────────────────────
// ACTIVE LOW: a '1' bit = drive LOW, a '0' bit = release HIGH
const uint32_t PRBS_A = 0b10110100110010101111000001101001;
const uint32_t PRBS_B = 0b01001011001101010000111110010110;

// ── Walsh-Hadamard H8 ─────────────────────────────────────
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

// ── Severity ──────────────────────────────────────────────
enum Severity { SEV_PASS, SEV_WARN, SEV_FAIL };

// ── Per-wire result ───────────────────────────────────────
struct WireResult {
  Severity severity;
  bool openFault;
  bool openFaultReverse;
  bool idleBleed;
  bool shortWith[8];
  bool crossTo;
  int  crossTarget;
  int  seqBitErrors;
  int  seqBitErrorsRev;
  int  parBitErrors;
  int  totalBleed;
  bool analogValid;
  float avgVoltageActive;
  float avgVoltageIdle;
  float estimatedResistance;
  float propagationDelayUs;
  bool  intermittent;
  int   passCount;
};

WireResult results[8];
int faultMatrix[8][8];
bool passHistory[8][HISTORY_DEPTH];
int  historyIndex = 0;
int  totalRuns    = 0;

// ============================================================
//  SETUP
// ============================================================
void setup() {
//  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcdPrint("Cable Tester v3", "Initializing...");

  for (int i = 0; i < 8; i++) {
    pinMode(TX_PINS[i], INPUT_PULLUP);
    pinMode(RX_PINS[i], INPUT_PULLUP);
  }

  for (int w = 0; w < 8; w++)
    for (int r = 0; r < HISTORY_DEPTH; r++)
      passHistory[w][r] = false;

  delay(1000);
//  Serial.println(F("Cable Tester v3.5 ready."));
//  Serial.println(F("No external resistors — INPUT_PULLUP, active-low logic."));
//  Serial.print(F("SETTLE_US=")); Serial.print(SETTLE_US);
//  Serial.print(F("  TEST_REPEAT=")); Serial.println(TEST_REPEAT);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop() {
  lcdPrint("Cable Tester v3", "Testing...");
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

//  if (CSV_MODE) printResultsCSV();
//  else          printResultsSerial();

  displayResultsLCD();

  int failCount = 0, warnCount = 0;
  for (int i = 0; i < 8; i++) {
    if (results[i].severity == SEV_FAIL) failCount++;
    if (results[i].severity == SEV_WARN) warnCount++;
  }

  lcd.clear();
  if (failCount == 0 && warnCount == 0) {
    lcd.setCursor(0, 0); lcd.print("ALL 8 PASS  :-)");
  } else {
    lcd.setCursor(0, 0);
    if (failCount > 0) { lcd.print(failCount); lcd.print(" FAIL "); }
    if (warnCount > 0) { lcd.print(warnCount); lcd.print(" WARN"); }
  }
  lcd.setCursor(0, 1); lcd.print("Retest in 3s...");
  delay(3000);
}

// ============================================================
//  DRIVE / RELEASE helpers (active-low logic)
// ============================================================
inline void drivePin(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

inline void releasePin(int pin) {
  pinMode(pin, INPUT_PULLUP);
}

// ============================================================
//  PHASE 1 — SEQUENTIAL SINGLE-WIRE TEST
// ============================================================
void runSequentialTest(bool reverse) {
  const int* drivePins = reverse ? RX_PINS  : TX_PINS;
  const int* readPins  = reverse ? TX_PINS  : RX_PINS;

  for (int i = 0; i < 8; i++) {
    releasePin(drivePins[i]);
    releasePin(readPins[i]);
  }
  delayMicroseconds(SETTLE_US * 4);

  for (int w = 0; w < 8; w++) {

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

    // Per-idle-pin counters for short detection (glitch dead-band)
    int shortCount[8]      = {};
    int cleanCount[8]      = {}; // FIX: track consecutive clean reads per pin

    float propDelay = -1.0;
    if (!reverse) {
      propDelay = measurePropDelay(drivePins[w], readPins[w]);
    }

    for (int rep = 0; rep < TEST_REPEAT; rep++) {
      for (int pass = 0; pass < 2; pass++) {
        uint32_t pattern = (pass == 0) ? PRBS_A : PRBS_B;

        for (int bit = 31; bit >= 0; bit--) {
          bool sendLow = (pattern >> bit) & 1;

          if (sendLow) drivePin(drivePins[w]);
          else         releasePin(drivePins[w]);

          delayMicroseconds(SETTLE_US);

          bool rxState[8];
          for (int r = 0; r < 8; r++)
            rxState[r] = (digitalRead(readPins[r]) == LOW);

          bool expected = sendLow;
          if (rxState[w] != expected) {
            bitErrors++;
            if (sendLow && !rxState[w]) localOpen = true;
          }

          if (!reverse && RX_HAS_ANALOG[w]) {
            if (sendLow) {
              analogSumAct  += analogRead(readPins[w]);
              analogCntAct++;
            } else {
              analogSumIdle += analogRead(readPins[w]);
              analogCntIdle++;
            }
          }

          for (int r = 0; r < 8; r++) {
            if (r == w) continue;

            if (digitalRead(drivePins[r]) == LOW) localIdle = true;

            if (sendLow && rxState[r]) {
              // Unexpected LOW on idle pin while we are driving LOW
              shortCount[r]++;
              cleanCount[r] = 0;
            } else {
              // Require DECAY_CLEAN_READS consecutive clean reads to decay
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

    // Only commit shorts from sequential phase on the forward pass.
    // Walsh will add to shortWith[] separately in analyzeFaultMatrix().
    if (!reverse) {
      for (int r = 0; r < 8; r++)
        if (shortCount[r] >= SHORT_THRESHOLD)
          results[w].shortWith[r] = true;
    }

    if (!reverse) {
      results[w].seqBitErrors       = bitErrors;
      results[w].openFault          = localOpen;
      results[w].idleBleed          = localIdle;
      results[w].propagationDelayUs = propDelay;
      results[w].analogValid        = RX_HAS_ANALOG[w];

      if (RX_HAS_ANALOG[w] && analogCntAct > 0 && analogCntIdle > 0) {
        results[w].avgVoltageActive = (analogSumAct  / (float)analogCntAct)  * (VCC / 1023.0);
        results[w].avgVoltageIdle   = (analogSumIdle / (float)analogCntIdle) * (VCC / 1023.0);
        float va = results[w].avgVoltageActive;
        results[w].estimatedResistance = (va < (VCC - 0.1)) ?
          PULLUP_OHMS * va / (VCC - va) : -1.0;
      } else {
        results[w].avgVoltageActive    = -1.0;
        results[w].avgVoltageIdle      = -1.0;
        results[w].estimatedResistance = -1.0;
      }

    } else {
      results[w].seqBitErrorsRev = bitErrors;

      // FIX: only flag asymmetric open if forward was clean AND reverse has errors.
      // Without this, wires on digital-only pins (W7/W8) spuriously trigger this.
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
//  PROPAGATION DELAY
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
//  PHASE 2 — WALSH-HADAMARD PARALLEL TEST
//
//  FIX: The fault matrix logic is rewritten.
//  Old code counted (rxGot != expected) for ALL tx/rx combos,
//  which meant an idle TX + idle RX = expected false + got false
//  = "match" in theory, but the expected calculation was wrong
//  for off-diagonal idle pairs, causing phantom faults.
//
//  New logic:
//    - Diagonal (tx==rx): count mismatches as self-errors
//    - Off-diagonal: only count as bleed if RX is unexpectedly LOW
//      while that TX was actively driving LOW. Idle-idle is NOT a fault.
// ============================================================
void runWalshTest() {
  uint32_t patterns[2] = {PRBS_A, PRBS_B};

  for (int i = 0; i < 8; i++) releasePin(TX_PINS[i]);
  delayMicroseconds(SETTLE_US * 4);

  for (int phase = 0; phase < 8; phase++) {

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

          // Which TX pins should be LOW this cycle?
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
            // Diagonal: did this wire's own TX reach its own RX?
            if (rxState[rx] != txActive[rx]) {
              faultMatrix[rx][rx]++;
            }

            // Off-diagonal: is an idle RX being pulled LOW by another TX?
            // Only flag if RX is LOW but its own TX is NOT active.
            // FIX: skip if RX's own TX is active (expected LOW, not a fault).
            if (rxState[rx] && !txActive[rx]) {
              // Unexpected LOW — find which active TX is bleeding into this RX
              for (int tx = 0; tx < 8; tx++) {
                if (tx == rx) continue;
                if (txActive[tx]) {
                  faultMatrix[tx][rx]++;
                }
              }
            }
          }

        } // bit
      } // prbs
    } // repeat

    for (int tx = 0; tx < 8; tx++) releasePin(TX_PINS[tx]);
    delayMicroseconds(SETTLE_US * 2);
  } // phase

  for (int i = 0; i < 8; i++) releasePin(TX_PINS[i]);
}

// ============================================================
//  ANALYZE fault matrix
// ============================================================
void analyzeFaultMatrix() {
  int totalBits = 8 * 2 * 32 * TEST_REPEAT;

  // Snapshot the sequential shorts, then clear them.
  // We re-confirm each one below; only those confirmed by Walsh too survive.
  bool seqShorts[8][8];
  for (int w = 0; w < 8; w++)
    for (int r = 0; r < 8; r++) {
      seqShorts[w][r]         = results[w].shortWith[r];
      results[w].shortWith[r] = false; // cleared — must be re-earned
    }

  for (int w = 0; w < 8; w++) {
    results[w].parBitErrors = faultMatrix[w][w];
    results[w].totalBleed   = 0;

    for (int r = 0; r < 8; r++) {
      if (r == w) continue;
      if (faultMatrix[w][r] > 0) {
        results[w].totalBleed += faultMatrix[w][r];
        // Cross-wire: open on self + strong signal elsewhere = mis-wired
        if (results[w].openFault && faultMatrix[w][r] > (totalBits / 4)) {
          results[w].crossTo     = true;
          results[w].crossTarget = r;
        } else {
          // Hard SHORT only if ALL THREE agree:
          //   1. Sequential phase saw it (seqShorts)
          //   2. Walsh bleed is bidirectional (w->r AND r->w)
          //   3. Walsh bleed count is substantial (> 10% of totalBits)
          //      — rules out single-cycle coupling transients
          bool seqSaw    = seqShorts[w][r] && seqShorts[r][w];
          bool walshBidi = (faultMatrix[r][w] > 0);
          bool walshHeavy= (faultMatrix[w][r] > totalBits / 10);
          if (seqSaw && walshBidi && walshHeavy) {
            results[w].shortWith[r] = true;
          }
          // Otherwise: bleed only — totalBleed already incremented → WARN
        }
      }
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
    for (int r = 0; r < 8; r++) if (results[w].shortWith[r]) anyShort = true;

    bool hardFail =
      results[w].openFault                           ||
      results[w].crossTo                             ||
      anyShort                                       ||
      results[w].seqBitErrors  > (seqTotalBits / 4) ||
      results[w].parBitErrors  > 0;

    bool warnFlag =
      results[w].openFaultReverse                    ||
      results[w].intermittent                        ||
      results[w].idleBleed                           ||
      results[w].seqBitErrors   > 0                  ||
      results[w].seqBitErrorsRev > 0                 ||
      results[w].totalBleed     > BLEED_WARN_MIN      ||
      (results[w].analogValid && results[w].avgVoltageIdle >= 0 &&
          results[w].avgVoltageIdle < WARN_V_HIGH_MIN)           ||
      (results[w].analogValid && results[w].avgVoltageActive >= 0 &&
          results[w].avgVoltageActive > WARN_V_LOW_MAX)          ||
      (results[w].analogValid && results[w].estimatedResistance > WARN_R_MAX) ||
      (results[w].propagationDelayUs > 0 &&
          results[w].propagationDelayUs > WARN_DELAY_MAX);

    if (hardFail)      results[w].severity = SEV_FAIL;
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
    int passes = 0, runs = min(totalRuns, HISTORY_DEPTH);
    for (int r = 0; r < runs; r++) passes += passHistory[w][r] ? 1 : 0;
    results[w].passCount    = passes;
    results[w].intermittent = (passes > 0 && passes < runs);
  }
  historyIndex = (historyIndex + 1) % HISTORY_DEPTH;
}

// ============================================================
//  LCD DISPLAY
// ============================================================

void displayResultsLCD() {
  for (int w = 0; w < 8; w++) {
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("W"); lcd.print(w + 1); lcd.print(":");
    switch (results[w].severity) {
      case SEV_PASS: lcd.print("PASS"); break;
      case SEV_WARN: lcd.print("WARN"); break;
      case SEV_FAIL: lcd.print("FAIL"); break;
    }
    if (totalRuns > 1) {
      char hist[7];
      snprintf(hist, sizeof(hist), " %d/%d", results[w].passCount, min(totalRuns, HISTORY_DEPTH));
      int col = 16 - strlen(hist);
      if (col > 6) { lcd.setCursor(col, 0); lcd.print(hist); }
    }

    lcd.setCursor(0, 1);

    if (results[w].severity == SEV_PASS) {
      if (results[w].analogValid && results[w].avgVoltageIdle > 0) {
        lcd.print(results[w].avgVoltageIdle, 2);
        lcd.print("V ");
        if (results[w].estimatedResistance >= 0) {
          lcd.print((int)results[w].estimatedResistance);
          lcd.print("ohm");
        }
      } else {
        lcd.print("Signal OK");
        if (results[w].propagationDelayUs > 0) {
          lcd.print(" ");
          lcd.print((int)results[w].propagationDelayUs);
          lcd.print("us");
        }
      }

    } else if (results[w].crossTo) {
      lcd.print("Wired to W");
      lcd.print(results[w].crossTarget + 1);
      lcd.print(" instead");

    } else if (results[w].openFault) {
      if (results[w].propagationDelayUs < 0)
        lcd.print("No signal-check");
      else
        lcd.print("Noisy-bad crimp?");

    } else if (results[w].openFaultReverse) {
      lcd.print("Asym open-diode?");

    } else {
      bool printedShort = false;
      for (int r = 0; r < 8; r++) {
        if (results[w].shortWith[r]) {
          if (!printedShort) {
            lcd.print("Short to W");
            lcd.print(r + 1);
            printedShort = true;
          } else {
            lcd.print("+");
            lcd.print(r + 1);
          }
        }
      }
      if (!printedShort) {
        if (results[w].idleBleed)
          lcd.print("Board leakage!");
        else if (results[w].intermittent)
          lcd.print("Loose-wiggle it");
        else if (results[w].analogValid &&
                 results[w].avgVoltageIdle > 0 &&
                 results[w].avgVoltageIdle < WARN_V_HIGH_MIN) {
          lcd.print("Low V:");
          lcd.print(results[w].avgVoltageIdle, 2);
          lcd.print("V");
        } else if (results[w].analogValid &&
                   results[w].estimatedResistance > WARN_R_MAX) {
          lcd.print("Hi-R:");
          lcd.print((int)results[w].estimatedResistance);
          lcd.print("ohm");
        } else if (results[w].propagationDelayUs > WARN_DELAY_MAX) {
          lcd.print("Slow:");
          lcd.print((int)results[w].propagationDelayUs);
          lcd.print("us");
        } else {
          int errs = results[w].seqBitErrors + results[w].seqBitErrorsRev;
          if (errs > 0) { lcd.print("Marginal:"); lcd.print(errs); lcd.print("err"); }
          else            lcd.print("Check signal");
        }
      }
    }

    delay(PAUSE_MS);
  }
}

/*

// ============================================================
//  SERIAL REPORT — human readable
// ============================================================
void printResultsSerial() {
  Serial.println(F("\n============================================================"));
  Serial.println(F("  Cable Tester v3.5 — Full Report"));
  Serial.print(F("  Run #")); Serial.print(totalRuns);
  Serial.print(F("  SETTLE_US=")); Serial.print(SETTLE_US);
  Serial.print(F("  TEST_REPEAT=")); Serial.println(TEST_REPEAT);
  Serial.println(F("  Logic: ACTIVE LOW (INPUT_PULLUP, no external resistors)"));
  Serial.println(F("============================================================"));

  int seqTotalBits = 2 * 32 * TEST_REPEAT;

  for (int w = 0; w < 8; w++) {
    Serial.print(F("Wire ")); Serial.print(w + 1);
    Serial.print(F("  TX:D")); Serial.print(TX_PINS[w]);
    Serial.print(F("  RX:"));
    if (RX_PINS[w] >= A0 && RX_PINS[w] <= A5) {
      Serial.print(F("A")); Serial.print(RX_PINS[w] - A0);
    } else {
      Serial.print(F("D")); Serial.print(RX_PINS[w]);
    }
    Serial.print(F("  -->  "));
    switch (results[w].severity) {
      case SEV_PASS: Serial.print(F("PASS")); break;
      case SEV_WARN: Serial.print(F("WARN")); break;
      case SEV_FAIL: Serial.print(F("FAIL")); break;
    }
    if (totalRuns > 1) {
      Serial.print(F("  (history: "));
      Serial.print(results[w].passCount);
      Serial.print(F("/")); Serial.print(min(totalRuns, HISTORY_DEPTH));
      Serial.print(F(")"));
    }
    Serial.println();

    if (results[w].openFault)
      Serial.println(F("    [OPEN]         No signal in forward direction"));
    if (results[w].openFaultReverse)
      Serial.println(F("    [OPEN-REV]     No signal in reverse — asymmetric (diode? one-way contact?)"));
    if (results[w].crossTo) {
      Serial.print(F("    [CROSS]        Signal at Wire "));
      Serial.print(results[w].crossTarget + 1);
      Serial.println(F(" instead — mis-wired connector"));
    }
    for (int r = 0; r < 8; r++) {
      if (results[w].shortWith[r]) {
        Serial.print(F("    [SHORT]        Shorted with Wire ")); Serial.println(r + 1);
      }
    }
    if (results[w].idleBleed)
      Serial.println(F("    [IDLE-BLEED]   Leakage on idle pin — check Nano for solder bridge"));
    if (results[w].intermittent)
      Serial.println(F("    [INTERMITTENT] Passed some runs, failed others — suspect loose pin"));

    Serial.print(F("    Seq errors fwd: ")); Serial.print(results[w].seqBitErrors);
    Serial.print(F(" / ")); Serial.println(seqTotalBits);
    Serial.print(F("    Seq errors rev: ")); Serial.print(results[w].seqBitErrorsRev);
    Serial.print(F(" / ")); Serial.println(seqTotalBits);
    Serial.print(F("    Walsh errors:   ")); Serial.println(results[w].parBitErrors);
    Serial.print(F("    Walsh bleed:    ")); Serial.println(results[w].totalBleed);

    if (results[w].analogValid) {
      Serial.print(F("    Idle voltage:   "));
      Serial.print(results[w].avgVoltageIdle, 3);
      Serial.println(F("V  (ideal: 5.000V)"));
      Serial.print(F("    Active voltage: "));
      Serial.print(results[w].avgVoltageActive, 3);
      Serial.println(F("V  (ideal: 0.000V)"));
      if (results[w].estimatedResistance >= 0) {
        Serial.print(F("    Est. resistance:")); Serial.print(results[w].estimatedResistance, 1); Serial.println(F(" ohm"));
      } else {
        Serial.println(F("    Est. resistance: N/A (open)"));
      }
    } else {
      Serial.println(F("    Voltage/R: N/A (digital-only RX pin)"));
    }

    if (results[w].propagationDelayUs >= 0) {
      Serial.print(F("    Prop. delay:    ")); Serial.print(results[w].propagationDelayUs, 1); Serial.println(F(" us"));
    } else {
      Serial.println(F("    Prop. delay:    N/A (no signal)"));
    }
    Serial.println();
  }

  Serial.println(F("--- Walsh fault matrix (TX row -> RX col, dot=clean) ---"));
  Serial.print(F("         "));
  for (int r = 0; r < 8; r++) { Serial.print(F("RX")); Serial.print(r+1); Serial.print(F("  ")); }
  Serial.println();
  for (int tx = 0; tx < 8; tx++) {
    Serial.print(F("TX")); Serial.print(tx+1); Serial.print(F(":    "));
    for (int rx = 0; rx < 8; rx++) {
      int v = faultMatrix[tx][rx];
      if      (v == 0)   Serial.print(F("  .  "));
      else if (v < 10)  { Serial.print(F("  ")); Serial.print(v); Serial.print(F("  ")); }
      else if (v < 100) { Serial.print(F(" "));  Serial.print(v); Serial.print(F("  ")); }
      else              { Serial.print(v);        Serial.print(F("  ")); }
    }
    Serial.println();
  }
  Serial.println(F("============================================================\n"));
}

// ============================================================
//  SERIAL REPORT — CSV
// ============================================================
void printResultsCSV() {
  if (totalRuns == 1) {
    Serial.println(F("run,wire,severity,open_fwd,open_rev,cross,cross_target,"
                     "short_mask,idle_bleed,intermittent,"
                     "seq_errors_fwd,seq_errors_rev,walsh_errors,walsh_bleed,"
                     "analog_valid,v_idle,v_active,resistance_ohm,"
                     "prop_delay_us,pass_count,history_depth"));
  }
  for (int w = 0; w < 8; w++) {
    Serial.print(totalRuns);                                       Serial.print(F(","));
    Serial.print(w + 1);                                           Serial.print(F(","));
    switch (results[w].severity) {
      case SEV_PASS: Serial.print(F("PASS")); break;
      case SEV_WARN: Serial.print(F("WARN")); break;
      case SEV_FAIL: Serial.print(F("FAIL")); break;
    }                                                              Serial.print(F(","));
    Serial.print(results[w].openFault ? 1 : 0);                    Serial.print(F(","));
    Serial.print(results[w].openFaultReverse ? 1 : 0);             Serial.print(F(","));
    Serial.print(results[w].crossTo ? 1 : 0);                      Serial.print(F(","));
    Serial.print(results[w].crossTo ? results[w].crossTarget+1:0); Serial.print(F(","));
    uint8_t sm = 0;
    for (int r = 0; r < 8; r++) if (results[w].shortWith[r]) sm |= (1 << r);
    Serial.print(sm);                                              Serial.print(F(","));
    Serial.print(results[w].idleBleed ? 1 : 0);                    Serial.print(F(","));
    Serial.print(results[w].intermittent ? 1 : 0);                 Serial.print(F(","));
    Serial.print(results[w].seqBitErrors);                         Serial.print(F(","));
    Serial.print(results[w].seqBitErrorsRev);                      Serial.print(F(","));
    Serial.print(results[w].parBitErrors);                         Serial.print(F(","));
    Serial.print(results[w].totalBleed);                           Serial.print(F(","));
    Serial.print(results[w].analogValid ? 1 : 0);                  Serial.print(F(","));
    Serial.print(results[w].avgVoltageIdle,   3);                  Serial.print(F(","));
    Serial.print(results[w].avgVoltageActive, 3);                  Serial.print(F(","));
    Serial.print(results[w].estimatedResistance, 1);               Serial.print(F(","));
    Serial.print(results[w].propagationDelayUs,  1);               Serial.print(F(","));
    Serial.print(results[w].passCount);                            Serial.print(F(","));
    Serial.println(min(totalRuns, HISTORY_DEPTH));
  }
}

*/

// ============================================================
//  HELPERS
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
