#include <Arduino.h>
#include <SPI.h>

#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   5
#define LORA_RST  27
#define LORA_DIO0 26

uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
  return v;
}

void writeReg(uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_SS, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(LORA_SS, HIGH);
  SPI.endTransaction();
}

void resetChip() {
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(20);
}

// Put every pin back to a known state at the start of each cycle
void initPins() {
  SPI.end();
  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  pinMode(LORA_DIO0, INPUT);
  pinMode(LORA_MISO, INPUT);
  pinMode(LORA_MOSI, INPUT);
  pinMode(LORA_SCK, INPUT);
  delay(5);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
}

// P: pin tests with SPI turned off. NSS is HIGH, so the chip leaves MISO floating.
int pinTests() {
  int fails = 0;
  SPI.end();
  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);

  // P1: MISO should follow the internal pull-up and pull-down
  pinMode(LORA_MISO, INPUT_PULLUP);
  delay(5);
  int up = digitalRead(LORA_MISO);
  pinMode(LORA_MISO, INPUT_PULLDOWN);
  delay(5);
  int down = digitalRead(LORA_MISO);
  Serial.printf("#   P1 MISO pull-up reads %d, pull-down reads %d (expect 1 and 0)\n", up, down);
  if (up == 1 && down == 0) {
    Serial.println("#      P1: PASS (MISO is not stuck)");
  } else if (up == 0 && down == 0) {
    Serial.println("#      P1: FAIL - MISO stuck LOW (short to GND, or chip driving it low)");
    fails++;
  } else if (up == 1 && down == 1) {
    Serial.println("#      P1: FAIL - MISO stuck HIGH (short to 3V3)");
    fails++;
  } else {
    Serial.println("#      P1: FAIL - odd reading (loose or noisy wire)");
    fails++;
  }

  // P2: MOSI to MISO leak. Drive MOSI, MISO must not follow.
  pinMode(LORA_MOSI, OUTPUT);
  pinMode(LORA_MISO, INPUT_PULLDOWN);
  digitalWrite(LORA_MOSI, HIGH);
  delay(5);
  int a = digitalRead(LORA_MISO);
  pinMode(LORA_MISO, INPUT_PULLUP);
  digitalWrite(LORA_MOSI, LOW);
  delay(5);
  int b = digitalRead(LORA_MISO);
  Serial.printf("#   P2 MOSI->MISO: MOSI high gives %d, MOSI low gives %d (expect 0 and 1)\n", a, b);
  if (a == 0 && b == 1) Serial.println("#      P2: PASS (no MOSI-MISO bridge)");
  else { Serial.println("#      P2: FAIL - MISO follows MOSI (solder bridge or leak)"); fails++; }
  pinMode(LORA_MOSI, INPUT);

  // P3: SCK to MISO leak
  pinMode(LORA_SCK, OUTPUT);
  pinMode(LORA_MISO, INPUT_PULLDOWN);
  digitalWrite(LORA_SCK, HIGH);
  delay(5);
  a = digitalRead(LORA_MISO);
  pinMode(LORA_MISO, INPUT_PULLUP);
  digitalWrite(LORA_SCK, LOW);
  delay(5);
  b = digitalRead(LORA_MISO);
  Serial.printf("#   P3 SCK->MISO: SCK high gives %d, SCK low gives %d (expect 0 and 1)\n", a, b);
  if (a == 0 && b == 1) Serial.println("#      P3: PASS (no SCK-MISO bridge)");
  else { Serial.println("#      P3: FAIL - MISO follows SCK (solder bridge or leak)"); fails++; }
  pinMode(LORA_SCK, INPUT);

  pinMode(LORA_MISO, INPUT);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  return fails;
}

bool testDio0() {
  resetChip();
  writeReg(0x01, 0x80);
  writeReg(0x01, 0x81);
  writeReg(0x40, 0x80);   // DIO0 = CadDone
  writeReg(0x12, 0xFF);
  delay(2);
  bool idleLow = (digitalRead(LORA_DIO0) == LOW);

  writeReg(0x01, 0x87);   // start channel activity detection
  bool pinHigh = false, flag = false;
  unsigned long t = millis();
  while (millis() - t < 300) {
    if (digitalRead(LORA_DIO0) == HIGH) pinHigh = true;
    if (readReg(0x12) & 0x04) {
      flag = true;
      if (digitalRead(LORA_DIO0) == HIGH) pinHigh = true;
      break;
    }
  }
  writeReg(0x12, 0xFF);
  writeReg(0x01, 0x80);

  if (!flag) {
    Serial.println("#   T5 DIO0: FAIL - the chip did not finish its test (chip not answering)");
    return false;
  }
  if (!idleLow) {
    Serial.println("#   T5 DIO0: FAIL - pin is HIGH when idle (short to 3V3, or wrong pin)");
    return false;
  }
  if (!pinHigh) {
    Serial.println("#   T5 DIO0: FAIL - chip says done but GPIO 26 did not go HIGH (DIO0 wire)");
    return false;
  }
  Serial.println("#   T5 DIO0: PASS");
  return true;
}

void runTests() {
  int fails = 0;
  initPins();
  Serial.println("# ===== WIRING TEST (all tests run) =====");

  // Pin tests first (no chip needed)
  fails += pinTests();

  resetChip();
  writeReg(0x01, 0x80);
  delay(5);

  // T1: chip version
  uint8_t v = readReg(0x42);
  Serial.printf("#   T1 version = 0x%02X (expect 0x12)\n", v);
  if (v == 0x12) {
    Serial.println("#   T1: PASS");
  } else {
    fails++;
    if (v == 0x00) Serial.println("#      T1: FAIL 0x00 - MISO held low, or no 3V3/GND, or NSS wire");
    else if (v == 0xFF) Serial.println("#      T1: FAIL 0xFF - MISO wire open, or NSS wire, or no power");
    else Serial.println("#      T1: FAIL unexpected value - check all SPI wires and 3V3");
  }

  // T2: stability
  int bad = 0;
  for (int i = 0; i < 300; i++) {
    if (readReg(0x42) != 0x12) bad++;
    delay(2);
  }
  Serial.printf("#   T2 stability: %d bad reads out of 300 - %s\n", bad, bad == 0 ? "PASS" : "FAIL");
  if (bad) fails++;

  // T3: write and read back
  writeReg(0x01, 0x80);
  writeReg(0x39, 0x5A);
  uint8_t a = readReg(0x39);
  writeReg(0x39, 0xA5);
  uint8_t b = readReg(0x39);
  bool t3 = (a == 0x5A && b == 0xA5);
  Serial.printf("#   T3 write/read: got 0x%02X and 0x%02X (expect 5A, A5) - %s\n", a, b, t3 ? "PASS" : "FAIL (MOSI, SCK or NSS wire)");
  if (!t3) fails++;

  // T4: RESET wire
  writeReg(0x39, 0x5A);
  resetChip();
  writeReg(0x01, 0x80);
  delay(2);
  uint8_t r = readReg(0x39);
  bool t4 = (r == 0x12);
  Serial.printf("#   T4 reset: 0x39 after reset = 0x%02X (expect 0x12) - %s\n", r, t4 ? "PASS" : "FAIL (RESET wire)");
  if (!t4) fails++;

  // T5: DIO0 wire
  if (!testDio0()) fails++;

  if (fails == 0) Serial.println("# RESULT: WIRING GOOD - this board is ready for the real sketch");
  else Serial.printf("# RESULT: PROBLEM - %d test(s) failed, see lines above\n", fails);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  runTests();
  Serial.println("# ----- repeating in 10 s -----");
  delay(10000);
}