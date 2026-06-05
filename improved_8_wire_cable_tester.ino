// ============================================================
//  8-Wire Cable Tester v3.1 — Arduino Nano + I2C 16x2 LCD
//
//  Test phases (in order):
//    1. Sequential single-wire test (isolation, both directions)
//    2. Walsh-Hadamard parallel test (system stress)
//
//  Features:
//    - Analog voltage sampling (signal quality / resistance est.)
//    - Propagation delay measurement per wire
//    - Intermittent fault detection across repeats
//    - Bidirectional test (TX→RX then RX→TX)
//    - Idle wire sniffing during sequential test
//    - Severity levels: PASS / WARN / FAIL
//    - Pass history across last HISTORY_DEPTH test runs
//    - Full fault matrix serial dump
//    - CSV serial output mode
//
//  Pin assignments:
//    TX (sequential):  D2–D9
//    RX (sequential):  A0, A1, A2, A3, D10, D13, D11, D12
//    I2C LCD SDA:      A4
//    I2C LCD SCL:      A5
//    NOTE: A6, A7 are analog-only on Nano — not used for digital
//    NOTE: analogRead() is only used on A0–A3 (wires 1–4).
//          Wires 5–8 use digital-only pins so voltage/resistance
//          readings are skipped for those wires.
//
//  Wiring:
//    - 10kΩ pull-down resistor from each RX pin to GND
//    - Both ends of the cable connector plug into the Nano
//    - LCD: VCC→5V, GND→GND, SDA→A4, SCL→A5
// ============================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ── LCD ───────────────────────────────────────────────────
// Try 0x3F if display stays blank
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ── Pin assignments ───────────────────────────────────────
const int TX_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const int RX_PINS[8] = {A0, A1, A2, A3, 10, 13, 11, 12};

// Which RX pins support analogRead() — only true analog pins
// Wires 0–3 use A0–A3, wires 4–7 use digital-only pins
const bool RX_HAS_ANALOG[8] = {true, true, true, true, false, false, false, false};

// ── Tuning ────────────────────────────────────────────────
const int     SETTLE_US      = 150;   // µs to wait after driving a pin
                                      // increase if you see false errors on long cables
const int     TEST_REPEAT    = 3;     // repeats per phase (increase for more thorough testing)
const int     PAUSE_MS       = 2500;  // ms to show each wire result on LCD
const int     HISTORY_DEPTH  = 10;    // how many full test runs to track for intermittent detection
const bool    CSV_MODE       = false; // set true to output CSV on serial instead of human-readable
const float   PULLDOWN_OHMS  = 10000.0; // pull-down resistor value in ohms
const float   VCC            = 5.0;     // supply voltage

// Thresholds for WARN
const float   WARN_V_HIGH_MIN  = VCC * 0.85; // below this → low signal WARN
const float   WARN_V_LOW_MAX   = VCC * 0.15; // above this → high idle WARN
const float   WARN_R_MAX       = 500.0;      // ohms — above this → high resistance WARN
const float   WARN_DELAY_MAX   = 1000.0;     // µs   — above this → slow prop delay WARN
// ─────────────────────────────────────────────────────────

// ── PRBS patterns ─────────────────────────────────────────
const uint32_t PRBS_A = 0b10110100110010101111000001101001;
const uint32_t PRBS_B = 0b01001011001101010000111110010110; // bitwise inverse

// ── Walsh-Hadamard H8 matrix ──────────────────────────────
// 8 orthogonal row patterns — each column has exactly 4 highs/lows
// This lets us drive all 8 wires simultaneously and still
// attribute any bleed to exactly one source wire.
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

  // Sequential faults
  bool openFault;            // no signal received in forward direction
  bool openFaultReverse;     // no signal in reverse direction (asymmetric)
  bool idleBleed;            // leakage on idle pins — possible board solder bridge
  bool shortWith[8];         // shorted to wire r
  bool crossTo;              // signal arrived at wrong RX pin
  int  crossTarget;          // which wrong pin
  int  seqBitErrors;         // bit errors, forward sequential test
  int  seqBitErrorsRev;      // bit errors, reverse sequential test

  // Parallel (Walsh) faults
  int  parBitErrors;         // errors on own path in Walsh test
  int  totalBleed;           // bleed onto other wires in Walsh test

  // Analog / quality (only valid for wires with analog-capable RX pins)
  bool  analogValid;         // true if analog readings are available
  float avgVoltageHigh;      // average voltage when driven HIGH (ideal = VCC)
  float avgVoltageLow;       // average voltage when driven LOW  (ideal = 0V)
  float estimatedResistance; // estimated wire resistance in ohms
  float propagationDelayUs;  // measured prop delay in µs (-1 = open/timeout)

  // Intermittent tracking
  bool  intermittent;        // passed some runs but failed others
  int   passCount;           // passes out of min(totalRuns, HISTORY_DEPTH)
};

WireResult results[8];

// ── Fault matrix (Walsh parallel test) ───────────────────
// faultMatrix[tx][rx] = number of unexpected signal events
// Diagonal = errors on correct path (open indicator)
// Off-diagonal = bleed (short/cross indicator)
int faultMatrix[8][8];

// ── Pass history ──────────────────────────────────────────
bool passHistory[8][HISTORY_DEPTH];
int  historyIndex = 0;
int  totalRuns    = 0;

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcdPrint("Cable Tester v3", "Initializing...");

  for (int i = 0; i < 8; i++) {
    pinMode(TX_PINS[i], OUTPUT);
    digitalWrite(TX_PINS[i], LOW);
    pinMode(RX_PINS[i], INPUT);
  }

  for (int w = 0; w < 8; w++)
    for (int r = 0; r < HISTORY_DEPTH; r++)
      passHistory[w][r] = false;

  delay(1000);
  Serial.println(F("Cable Tester v3.1 ready."));
  Serial.print(F("SETTLE_US=")); Serial.print(SETTLE_US);
  Serial.print(F("  TEST_REPEAT=")); Serial.println(TEST_REPEAT);
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

  if (CSV_MODE) printResultsCSV();
  else          printResultsSerial();

  displayResultsLCD();

  // Summary screen
  int failCount = 0, warnCount = 0;
  for (int i = 0; i < 8; i++) {
    if (results[i].severity == SEV_FAIL) failCount++;
    if (results[i].severity == SEV_WARN) warnCount++;
  }

  lcd.clear();
  if (failCount == 0 && warnCount == 0) {
    lcd.setCursor(0, 0); lcd.print("ALL 8 PASS  :-)");
    lcd.setCursor(0, 1); lcd.print("Retest in 3s...");
  } else {
    lcd.setCursor(0, 0);
    if (failCount > 0) { lcd.print(failCount); lcd.print(" FAIL "); }
    if (warnCount > 0) { lcd.print(warnCount); lcd.print(" WARN"); }
    lcd.setCursor(0, 1); lcd.print("Retest in 3s...");
  }
  delay(3000);
}

// ============================================================
//  PHASE 1 — SEQUENTIAL SINGLE-WIRE TEST
//
//  reverse=false: TX_PINS drive,  RX_PINS read  (normal)
//  reverse=true:  RX_PINS drive,  TX_PINS read  (reverse)
//
//  FIX: propagation delay is now measured in a dedicated call
//  BEFORE the main bit loop, so it does not interfere with
//  the bit-error counting. Previously measurePropDelay() drove
//  the pin itself and left it LOW, causing the first bit of
//  every wire to read wrong and falsely flag every wire as OPEN.
// ============================================================
void runSequentialTest(bool reverse) {
  const int* drivePins = reverse ? RX_PINS  : TX_PINS;
  const int* readPins  = reverse ? TX_PINS  : RX_PINS;

  for (int i = 0; i < 8; i++) {
    pinMode(drivePins[i], OUTPUT);
    digitalWrite(drivePins[i], LOW);
    pinMode(readPins[i], INPUT);
  }
  delayMicroseconds(SETTLE_US * 4); // let pins settle before starting

  for (int w = 0; w < 8; w++) {

    // Progress bar
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(reverse ? "Rev wire " : "Fwd wire ");
    lcd.print(w + 1);
    lcd.setCursor(0, 1);
    lcd.print("[");
    for (int b = 0; b < w * 2; b++) lcd.print("=");
    lcd.print(">");

    int   bitErrors     = 0;
    long  analogSumH    = 0;
    long  analogSumL    = 0;
    int   analogCountH  = 0;
    int   analogCountL  = 0;
    bool  localShort[8] = {};
    bool  localOpen     = false;
    bool  localIdle     = false;

    // ── Measure propagation delay BEFORE the bit loop ──────
    // This is done separately so it doesn't corrupt bit timing.
    // Only measured in forward direction (reverse uses same wire).
    float propDelay = -1.0;
    if (!reverse) {
      propDelay = measurePropDelay(drivePins[w], readPins[w]);
      // Pin is now LOW again and settled — safe to start bit loop
    }

    // ── Main bit loop ───────────────────────────────────────
    for (int rep = 0; rep < TEST_REPEAT; rep++) {
      for (int pass = 0; pass < 2; pass++) {
        uint32_t pattern = (pass == 0) ? PRBS_A : PRBS_B;

        for (int bit = 31; bit >= 0; bit--) {
          bool sendHigh = (pattern >> bit) & 1;

          digitalWrite(drivePins[w], sendHigh ? HIGH : LOW);
          delayMicroseconds(SETTLE_US);

          // Read all pins
          bool rxState[8];
          for (int r = 0; r < 8; r++) {
            rxState[r] = digitalRead(readPins[r]);
          }

          // Check expected pin
          if (rxState[w] != sendHigh) {
            bitErrors++;
            if (sendHigh && !rxState[w]) localOpen = true;
          }

          // Analog sampling — only on pins that support it
          // FIX: analogRead() on digital-only pins (D10/D11/D12/D13)
          // returns garbage on the Nano. Only sample A0–A3.
          if (!reverse && RX_HAS_ANALOG[w]) {
            if (sendHigh) {
              analogSumH += analogRead(readPins[w]);
              analogCountH++;
            } else {
              analogSumL += analogRead(readPins[w]);
              analogCountL++;
            }
          }

          // Idle wire sniffer
          for (int r = 0; r < 8; r++) {
            if (r == w) continue;
            if (digitalRead(drivePins[r]) == HIGH) localIdle = true;
            if (rxState[r] && sendHigh)            localShort[r] = true;
          }

        } // bit
      } // pass
    } // repeat

    // Clean up
    digitalWrite(drivePins[w], LOW);
    delayMicroseconds(SETTLE_US * 2);

    // Store results
    if (!reverse) {
      results[w].seqBitErrors       = bitErrors;
      results[w].openFault          = localOpen;
      results[w].idleBleed          = localIdle;
      results[w].propagationDelayUs = propDelay;
      results[w].analogValid        = RX_HAS_ANALOG[w];

      if (RX_HAS_ANALOG[w] && analogCountH > 0 && analogCountL > 0) {
        results[w].avgVoltageHigh = (analogSumH / (float)analogCountH) * (VCC / 1023.0);
        results[w].avgVoltageLow  = (analogSumL / (float)analogCountL) * (VCC / 1023.0);
        // Voltage divider: R_wire = R_pull × (Vcc - Vrx) / Vrx
        float vh = results[w].avgVoltageHigh;
        results[w].estimatedResistance = (vh > 0.1) ? PULLDOWN_OHMS * (VCC - vh) / vh : -1.0;
      } else {
        results[w].avgVoltageHigh     = -1.0;
        results[w].avgVoltageLow      = -1.0;
        results[w].estimatedResistance= -1.0;
      }

      for (int r = 0; r < 8; r++)
        if (localShort[r]) results[w].shortWith[r] = true;

    } else {
      results[w].seqBitErrorsRev  = bitErrors;
      results[w].openFaultReverse = localOpen;
    }
  } // wire loop

  // Restore known state
  for (int i = 0; i < 8; i++) {
    pinMode(TX_PINS[i], OUTPUT);
    digitalWrite(TX_PINS[i], LOW);
    pinMode(RX_PINS[i], INPUT);
  }
  delayMicroseconds(SETTLE_US * 4);
}

// ============================================================
//  PROPAGATION DELAY MEASUREMENT
//  Drives txPin HIGH and polls rxPin until it responds.
//  Returns elapsed microseconds, or -1.0 on timeout (open).
//  Leaves txPin LOW and settled when it returns.
//  Called BEFORE the main bit loop so it doesn't affect
//  bit-error counting.
// ============================================================
float measurePropDelay(int txPin, int rxPin) {
  digitalWrite(txPin, LOW);
  delayMicroseconds(300); // ensure clean LOW baseline

  unsigned long t0       = micros();
  unsigned long deadline = t0 + 5000UL; // 5ms timeout

  digitalWrite(txPin, HIGH);

  while (micros() < deadline) {
    if (digitalRead(rxPin) == HIGH) {
      float d = (float)(micros() - t0);
      digitalWrite(txPin, LOW);
      delayMicroseconds(SETTLE_US * 2);
      return d;
    }
  }

  // Timeout — wire is open or very slow
  digitalWrite(txPin, LOW);
  delayMicroseconds(SETTLE_US * 2);
  return -1.0;
}

// ============================================================
//  PHASE 2 — WALSH-HADAMARD PARALLEL TEST
//  All 8 TX pins driven simultaneously with orthogonal patterns.
//  faultMatrix[tx][rx] accumulates error counts.
// ============================================================
void runWalshTest() {
  uint32_t patterns[2] = {PRBS_A, PRBS_B};

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
          bool bitVal = (patterns[p] >> bit) & 1;

          for (int tx = 0; tx < 8; tx++) {
            bool txHigh = bitVal && ((WALSH[tx] >> (7 - phase)) & 1);
            digitalWrite(TX_PINS[tx], txHigh ? HIGH : LOW);
          }

          delayMicroseconds(SETTLE_US);

          bool rxState[8];
          for (int rx = 0; rx < 8; rx++)
            rxState[rx] = digitalRead(RX_PINS[rx]);

          // Decode: score every TX→RX combination
          for (int rx = 0; rx < 8; rx++) {
            bool rxGot = rxState[rx];
            for (int tx = 0; tx < 8; tx++) {
              bool txWasHigh = bitVal && ((WALSH[tx] >> (7 - phase)) & 1);
              bool expected  = (tx == rx) && txWasHigh;
              if (rxGot != expected) {
                if (tx == rx) {
                  // Correct path mismatch → open or integrity fault
                  faultMatrix[tx][rx]++;
                } else if (rxGot && txWasHigh) {
                  // Different TX was HIGH and this RX picked it up → bleed
                  faultMatrix[tx][rx]++;
                }
              }
            }
          }

        } // bit
      } // prbs pass
    } // repeat

    for (int tx = 0; tx < 8; tx++) digitalWrite(TX_PINS[tx], LOW);
    delayMicroseconds(SETTLE_US * 2);
  } // phase
}

// ============================================================
//  ANALYZE fault matrix → wire results
// ============================================================
void analyzeFaultMatrix() {
  int totalBits = 8 * 2 * 32 * TEST_REPEAT;

  for (int w = 0; w < 8; w++) {
    results[w].parBitErrors = faultMatrix[w][w];
    results[w].totalBleed   = 0;

    for (int r = 0; r < 8; r++) {
      if (r == w) continue;
      if (faultMatrix[w][r] > 0) {
        results[w].totalBleed += faultMatrix[w][r];

        if (results[w].openFault && faultMatrix[w][r] > (totalBits / 4)) {
          // Heavy bleed on a dead wire → cross-wired, not just a short
          results[w].crossTo     = true;
          results[w].crossTarget = r;
        } else {
          results[w].shortWith[r] = true;
        }
      }
    }
  }
}

// ============================================================
//  COMPUTE SEVERITY
//
//  FAIL — hard fault: the wire cannot be trusted at all
//    - Open in forward direction
//    - Cross-wired (signal arrives at wrong pin)
//    - Shorted to another wire
//    - Heavy bit errors in sequential or Walsh test
//
//  WARN — marginal: wire works but something is off
//    - Open in reverse direction only (asymmetric — investigate)
//    - Low signal voltage (< 85% Vcc) — corroded crimp?
//    - High idle voltage (> 15% Vcc) — resistive leakage?
//    - Estimated resistance > 500Ω
//    - Propagation delay > 1ms
//    - Any bit errors (but below hard-fail threshold)
//    - Intermittent (passes some runs, fails others)
//    - Idle bleed detected (possible board solder bridge)
//
//  FIX: openFaultReverse is now WARN, not FAIL.
//  A reverse-only open can be caused by pull-down asymmetry
//  during the test itself and is worth flagging but not
//  condemning the wire outright.
// ============================================================
void computeSeverity() {
  int seqTotalBits = 2 * 32 * TEST_REPEAT;

  for (int w = 0; w < 8; w++) {
    bool anyShort = false;
    for (int r = 0; r < 8; r++) if (results[w].shortWith[r]) anyShort = true;

    bool hardFail =
      results[w].openFault                            ||
      results[w].crossTo                               ||
      anyShort                                         ||
      results[w].seqBitErrors  > (seqTotalBits / 4)   ||
      results[w].parBitErrors  > 0;

    bool warnFlag =
      results[w].openFaultReverse                      ||
      results[w].intermittent                          ||
      results[w].idleBleed                             ||
      results[w].seqBitErrors   > 0                    ||
      results[w].seqBitErrorsRev > 0                   ||
      results[w].totalBleed     > 0                    ||
      (results[w].analogValid && results[w].avgVoltageHigh >= 0 &&
          results[w].avgVoltageHigh < WARN_V_HIGH_MIN) ||
      (results[w].analogValid && results[w].avgVoltageLow >= 0 &&
          results[w].avgVoltageLow  > WARN_V_LOW_MAX)  ||
      (results[w].analogValid && results[w].estimatedResistance > WARN_R_MAX) ||
      (results[w].propagationDelayUs > WARN_DELAY_MAX);

    if (hardFail)        results[w].severity = SEV_FAIL;
    else if (warnFlag)   results[w].severity = SEV_WARN;
    else                 results[w].severity = SEV_PASS;
  }
}

// ============================================================
//  UPDATE PASS HISTORY & INTERMITTENT DETECTION
// ============================================================
void updateHistory() {
  totalRuns = min(totalRuns + 1, HISTORY_DEPTH);

  for (int w = 0; w < 8; w++) {
    passHistory[w][historyIndex] = (results[w].severity == SEV_PASS);

    int passes = 0;
    int runs   = min(totalRuns, HISTORY_DEPTH);
    for (int r = 0; r < runs; r++) passes += passHistory[w][r] ? 1 : 0;

    results[w].passCount   = passes;
    results[w].intermittent = (passes > 0 && passes < runs);
  }

  historyIndex = (historyIndex + 1) % HISTORY_DEPTH;
}

// ============================================================
//  LCD DISPLAY
//  Line 1: W[n]: PASS/WARN/FAIL  [history e.g. 9/10]
//  Line 2: plain-English description of the most important finding
// ============================================================
void displayResultsLCD() {
  for (int w = 0; w < 8; w++) {
    lcd.clear();

    // ── Line 1 ──────────────────────────────────────────────
    lcd.setCursor(0, 0);
    lcd.print("W");
    lcd.print(w + 1);
    lcd.print(":");
    switch (results[w].severity) {
      case SEV_PASS: lcd.print("PASS"); break;
      case SEV_WARN: lcd.print("WARN"); break;
      case SEV_FAIL: lcd.print("FAIL"); break;
    }

    // History on right side of line 1 (only after >1 run)
    if (totalRuns > 1) {
      int runs = min(totalRuns, HISTORY_DEPTH);
      // e.g. " 9/10" — build right-aligned
      char hist[7];
      snprintf(hist, sizeof(hist), " %d/%d", results[w].passCount, runs);
      int col = 16 - strlen(hist);
      if (col > 6) { lcd.setCursor(col, 0); lcd.print(hist); }
    }

    // ── Line 2: plain-English description ───────────────────
    lcd.setCursor(0, 1);

    if (results[w].severity == SEV_PASS) {
      // Show voltage if available, otherwise just confirm clean
      if (results[w].analogValid && results[w].avgVoltageHigh > 0) {
        lcd.print(results[w].avgVoltageHigh, 2);
        lcd.print("V ");
        if (results[w].estimatedResistance >= 0 && results[w].estimatedResistance < 9999) {
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
      // Cross-wire is the most important fault — show it first
      lcd.print("Wired to W");
      lcd.print(results[w].crossTarget + 1);
      lcd.print(" instead");

    } else if (results[w].openFault) {
      if (results[w].propagationDelayUs < 0) {
        lcd.print("No signal-check");
        // Use remaining space for a hint
        // (16 chars total, "No signal-check" = 15 chars, space for 1 more)
      } else {
        // Signal present but too many bit errors — partial open / bad crimp
        lcd.print("Noisy-bad crimp?");
      }

    } else if (results[w].openFaultReverse) {
      lcd.print("Asym open-diode?");

    } else {
      // Check for shorts
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
        // WARN-level issues — pick the most actionable one
        if (results[w].idleBleed) {
          lcd.print("Board leakage!");
        } else if (results[w].intermittent) {
          lcd.print("Loose-wiggle it");
        } else if (results[w].analogValid &&
                   results[w].avgVoltageHigh > 0 &&
                   results[w].avgVoltageHigh < WARN_V_HIGH_MIN) {
          lcd.print("Low V:");
          lcd.print(results[w].avgVoltageHigh, 2);
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
        } else if (results[w].seqBitErrors > 0 || results[w].seqBitErrorsRev > 0) {
          int errs = results[w].seqBitErrors + results[w].seqBitErrorsRev;
          lcd.print("Marginal:");
          lcd.print(errs);
          lcd.print(" errs");
        } else {
          lcd.print("Check signal");
        }
      }
    }

    delay(PAUSE_MS);
  }
}

// ============================================================
//  SERIAL REPORT — human readable
// ============================================================
void printResultsSerial() {
  Serial.println(F("\n============================================================"));
  Serial.println(F("  Cable Tester v3.1 — Full Report"));
  Serial.print(F("  Test run #")); Serial.print(totalRuns);
  Serial.print(F("  SETTLE_US=")); Serial.print(SETTLE_US);
  Serial.print(F("  TEST_REPEAT=")); Serial.println(TEST_REPEAT);
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
      Serial.print(F("/"));
      Serial.print(min(totalRuns, HISTORY_DEPTH));
      Serial.print(F(")"));
    }
    Serial.println();

    if (results[w].openFault)
      Serial.println(F("    [OPEN]          No signal in forward direction"));
    if (results[w].openFaultReverse)
      Serial.println(F("    [OPEN-REV]       No signal in reverse direction — asymmetric fault (diode? one-way contact?)"));
    if (results[w].crossTo) {
      Serial.print(F("    [CROSS]          Signal arrived at Wire "));
      Serial.print(results[w].crossTarget + 1);
      Serial.println(F(" instead of expected RX pin — mis-wired connector"));
    }
    for (int r = 0; r < 8; r++) {
      if (results[w].shortWith[r]) {
        Serial.print(F("    [SHORT]          Shorted with Wire "));
        Serial.println(r + 1);
      }
    }
    if (results[w].idleBleed)
      Serial.println(F("    [IDLE-BLEED]     Leakage on idle pin — check for solder bridge on Nano"));
    if (results[w].intermittent)
      Serial.println(F("    [INTERMITTENT]   Passed some runs, failed others — suspect loose pin or hairline crack"));

    Serial.print(F("    Seq errors (fwd): ")); Serial.print(results[w].seqBitErrors);
    Serial.print(F(" / ")); Serial.println(seqTotalBits);
    Serial.print(F("    Seq errors (rev): ")); Serial.print(results[w].seqBitErrorsRev);
    Serial.print(F(" / ")); Serial.println(seqTotalBits);
    Serial.print(F("    Walsh errors:     ")); Serial.println(results[w].parBitErrors);
    Serial.print(F("    Walsh bleed:      ")); Serial.println(results[w].totalBleed);

    if (results[w].analogValid) {
      if (results[w].avgVoltageHigh >= 0) {
        Serial.print(F("    Avg HIGH voltage: ")); Serial.print(results[w].avgVoltageHigh, 3); Serial.println(F("V  (ideal: 5.000V)"));
        Serial.print(F("    Avg LOW  voltage: ")); Serial.print(results[w].avgVoltageLow,  3); Serial.println(F("V  (ideal: 0.000V)"));
      }
      if (results[w].estimatedResistance >= 0) {
        Serial.print(F("    Est. resistance:  ")); Serial.print(results[w].estimatedResistance, 1); Serial.println(F(" ohm"));
      } else {
        Serial.println(F("    Est. resistance:  N/A (open circuit)"));
      }
    } else {
      Serial.println(F("    Voltage/resistance: N/A (digital-only RX pin)"));
    }

    if (results[w].propagationDelayUs >= 0) {
      Serial.print(F("    Prop. delay:      ")); Serial.print(results[w].propagationDelayUs, 1); Serial.println(F(" us"));
    } else {
      Serial.println(F("    Prop. delay:      N/A (no signal received)"));
    }
    Serial.println();
  }

  // Walsh fault matrix
  Serial.println(F("--- Walsh fault matrix (TX row driving -> RX col reading) ---"));
  Serial.println(F("    A dot means zero errors. Numbers mean unexpected signal events."));
  Serial.print(F("         "));
  for (int r = 0; r < 8; r++) { Serial.print(F("RX")); Serial.print(r+1); Serial.print(F("  ")); }
  Serial.println();
  for (int tx = 0; tx < 8; tx++) {
    Serial.print(F("TX")); Serial.print(tx+1); Serial.print(F(":    "));
    for (int rx = 0; rx < 8; rx++) {
      int v = faultMatrix[tx][rx];
      if (v == 0)       Serial.print(F("  .  "));
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
                     "analog_valid,avg_v_high,avg_v_low,resistance_ohm,"
                     "prop_delay_us,pass_count,history_depth"));
  }
  for (int w = 0; w < 8; w++) {
    Serial.print(totalRuns);                                      Serial.print(F(","));
    Serial.print(w + 1);                                          Serial.print(F(","));
    switch (results[w].severity) {
      case SEV_PASS: Serial.print(F("PASS")); break;
      case SEV_WARN: Serial.print(F("WARN")); break;
      case SEV_FAIL: Serial.print(F("FAIL")); break;
    }                                                             Serial.print(F(","));
    Serial.print(results[w].openFault ? 1 : 0);                   Serial.print(F(","));
    Serial.print(results[w].openFaultReverse ? 1 : 0);            Serial.print(F(","));
    Serial.print(results[w].crossTo ? 1 : 0);                     Serial.print(F(","));
    Serial.print(results[w].crossTo ? results[w].crossTarget+1:0);Serial.print(F(","));
    uint8_t sm = 0;
    for (int r = 0; r < 8; r++) if (results[w].shortWith[r]) sm |= (1 << r);
    Serial.print(sm);                                             Serial.print(F(","));
    Serial.print(results[w].idleBleed ? 1 : 0);                   Serial.print(F(","));
    Serial.print(results[w].intermittent ? 1 : 0);                Serial.print(F(","));
    Serial.print(results[w].seqBitErrors);                        Serial.print(F(","));
    Serial.print(results[w].seqBitErrorsRev);                     Serial.print(F(","));
    Serial.print(results[w].parBitErrors);                        Serial.print(F(","));
    Serial.print(results[w].totalBleed);                          Serial.print(F(","));
    Serial.print(results[w].analogValid ? 1 : 0);                 Serial.print(F(","));
    Serial.print(results[w].avgVoltageHigh, 3);                   Serial.print(F(","));
    Serial.print(results[w].avgVoltageLow,  3);                   Serial.print(F(","));
    Serial.print(results[w].estimatedResistance, 1);              Serial.print(F(","));
    Serial.print(results[w].propagationDelayUs, 1);               Serial.print(F(","));
    Serial.print(results[w].passCount);                           Serial.print(F(","));
    Serial.println(min(totalRuns, HISTORY_DEPTH));
  }
}

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
    results[w].avgVoltageHigh      = -1.0;
    results[w].avgVoltageLow       = -1.0;
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
