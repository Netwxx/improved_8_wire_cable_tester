// ============================================================
//  8-Wire Cable Tester — Arduino Nano + I2C 16x2 LCD
// ============================================================
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD: address 0x27 is the most common — try 0x3F if blank
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int TX_PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
const int RX_PINS[8] = {A0, A1, A2, A3, 10, 13, 11, 12};

const uint32_t TEST_PATTERN = 0b10110100110010101111000001101001;
const int SETTLE_US   = 100;
const int TEST_REPEAT = 3;     // ← increase this for more thorough testing
const int PAUSE_MS    = 2000;  // how long to show each wire result on LCD

struct WireResult {
  bool passed;
  bool openFault;
  bool shortFault[8];
  bool crossFault;
  int  crossTarget;
  int  bitErrors;
};

WireResult results[8];

// ============================================================
void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcdPrint("Cable Tester", "Starting up...");
  delay(1500);

  for (int i = 0; i < 8; i++) {
    pinMode(TX_PINS[i], OUTPUT);
    digitalWrite(TX_PINS[i], LOW);
  }
  for (int i = 0; i < 8; i++) {
    pinMode(RX_PINS[i], INPUT);
  }

  delay(500);
}

// ============================================================
void loop() {
  lcdPrint("Insert cable,", "testing...");
  delay(1000);

  runAllTests();
  printResultsSerial();
  displayResultsLCD();

  // Brief summary before looping
  int failCount = 0;
  for (int i = 0; i < 8; i++) if (!results[i].passed) failCount++;

  if (failCount == 0) {
    lcdPrint("ALL PASS  :-)", "Next test in 3s");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(failCount);
    lcd.print(" WIRE(S) FAILED");
    lcd.setCursor(0, 1);
    lcd.print("Next test in 3s");
  }

  delay(3000);  // pause before auto-running again
}

// ============================================================
// Core test logic (unchanged from before)
// ============================================================
void runAllTests() {
  for (int w = 0; w < 8; w++) {
    results[w] = {true, false, {false,false,false,false,false,false,false,false}, false, -1, 0};

    int   totalBitErrors = 0;
    bool  anyOpen        = false;
    bool  anyShort[8]    = {};
    bool  anyCross       = false;
    int   crossTarget    = -1;

    // Show progress on LCD while testing
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Testing wire ");
    lcd.print(w + 1);
    lcd.setCursor(0, 1);
    lcd.print("[");
    // Draw a simple progress bar across the bottom row
    int barWidth = map(w, 0, 7, 0, 14);
    for (int b = 0; b < barWidth; b++) lcd.print("=");
    lcd.print(">");

    for (int rep = 0; rep < TEST_REPEAT; rep++) {
      for (int bit = 31; bit >= 0; bit--) {
        bool sendHigh = (TEST_PATTERN >> bit) & 1;

        digitalWrite(TX_PINS[w], sendHigh ? HIGH : LOW);
        delayMicroseconds(SETTLE_US);

        bool rxState[8];
        for (int r = 0; r < 8; r++) rxState[r] = digitalRead(RX_PINS[r]);

        if (rxState[w] != sendHigh) {
          totalBitErrors++;
          if (sendHigh && !rxState[w]) anyOpen = true;
          if (sendHigh) {
            for (int r = 0; r < 8; r++) {
              if (r != w && rxState[r]) { anyCross = true; crossTarget = r; }
            }
          }
        }

        if (sendHigh) {
          for (int r = 0; r < 8; r++) {
            if (r != w && rxState[r]) anyShort[r] = true;
          }
        }
      }
    }

    results[w].bitErrors   = totalBitErrors;
    results[w].openFault   = anyOpen;
    results[w].crossFault  = anyCross;
    results[w].crossTarget = crossTarget;
    for (int r = 0; r < 8; r++) results[w].shortFault[r] = anyShort[r];
    results[w].passed = (totalBitErrors == 0);

    digitalWrite(TX_PINS[w], LOW);
    delayMicroseconds(SETTLE_US);
  }
}

// ============================================================
// Show each wire result on the LCD one at a time
// ============================================================
void displayResultsLCD() {
  for (int w = 0; w < 8; w++) {
    lcd.clear();

    // Line 1: wire number + pass/fail
    lcd.setCursor(0, 0);
    lcd.print("Wire ");
    lcd.print(w + 1);
    lcd.print(": ");
    lcd.print(results[w].passed ? "PASS" : "FAIL");

    // Line 2: fault detail (first fault found wins display priority)
    lcd.setCursor(0, 1);
    if (results[w].passed) {
      lcd.print("All bits OK");
    } else if (results[w].openFault && !results[w].crossFault) {
      lcd.print("Open circuit");
    } else if (results[w].crossFault) {
      lcd.print("Cross->Wire ");
      lcd.print(results[w].crossTarget + 1);
    } else {
      // Find first short
      for (int r = 0; r < 8; r++) {
        if (results[w].shortFault[r]) {
          lcd.print("Short w/Wire ");
          lcd.print(r + 1);
          break;
        }
      }
    }

    delay(PAUSE_MS);
  }
}

// ============================================================
// Serial report (unchanged, still useful for debugging)
// ============================================================
void printResultsSerial() {
  Serial.println(F("\n========== TEST RESULTS =========="));
  bool allPass = true;

  for (int w = 0; w < 8; w++) {
    Serial.print(F("Wire "));
    Serial.print(w + 1);
    Serial.print(F("  →  "));

    if (results[w].passed) {
      Serial.println(F("PASS"));
    } else {
      allPass = false;
      if (results[w].openFault)  Serial.print(F("[OPEN] "));
      if (results[w].crossFault) { Serial.print(F("[CROSS->Wire ")); Serial.print(results[w].crossTarget + 1); Serial.print(F("] ")); }
      for (int r = 0; r < 8; r++) {
        if (results[w].shortFault[r]) { Serial.print(F("[SHORT w/Wire ")); Serial.print(r + 1); Serial.print(F("] ")); }
      }
      Serial.print(F("(")); Serial.print(results[w].bitErrors); Serial.println(F(" bit errors)"));
    }
  }

  Serial.println(allPass ? F("ALL WIRES GOOD") : F("FAULTS DETECTED"));
}

// ============================================================
// Helper: print two lines to LCD cleanly
// ============================================================
void lcdPrint(const char* line1, const char* line2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}