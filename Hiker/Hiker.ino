#include <Arduino.h>   // basic Arduino stuff
#include <SPI.h>       // SPI to talk to the radio chip
#include <LoRa.h>      // LoRa library

// Same wires as the hiker, since both boards are built the same way
#define LORA_SCK  18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_FREQ 915E6  // must match the hiker
#define TX_POWER  14     // must match the hiker too
#define LED_PIN   2      // onboard LED, blinks on init failure

#define HEARTBEAT_MS 5000   // print a "still alive" line every 5 s. Set to 0 to turn it off.

String serialLine = "";     // collects characters from the PC until I get a full line
unsigned long lastBeat = 0; // time of the last heartbeat line

// ---------- radio health ----------

// Reads one register from the LoRa chip (same function as the hiker)
uint8_t readReg(uint8_t reg) {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // slow 1 MHz
  digitalWrite(LORA_SS, LOW);     // select the chip
  SPI.transfer(reg & 0x7F);       // register address, "read" mode
  uint8_t v = SPI.transfer(0x00); // dummy byte, chip sends the value back
  digitalWrite(LORA_SS, HIGH);    // release the chip
  SPI.endTransaction();
  return v;
}

// Blink the LED n times
void blinkLed(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// A good SX1276 says 0x12 when I read the version. Anything else means a wiring problem.
String checkVersion() {
  uint8_t v = readReg(0x42);
  if (v == 0x12) return "";
  char msg[80];
  if (v == 0x00)      snprintf(msg, sizeof(msg), "version 0x00 (MISO stuck low, or no 3V3/GND, or NSS wire)");
  else if (v == 0xFF) snprintf(msg, sizeof(msg), "version 0xFF (MISO open, or NSS wire, or no power)");
  else                snprintf(msg, sizeof(msg), "version 0x%02X (expect 0x12, check SPI wires)", v);
  return String(msg);
}

// Compares what the chip really holds against what I asked for
String checkSettings() {
  // join the 3 frequency registers into one number
  uint32_t frf = ((uint32_t)readReg(0x06) << 16) | ((uint32_t)readReg(0x07) << 8) | readReg(0x08);
  uint8_t mc1 = readReg(0x1D);   // bandwidth and coding rate live here
  uint8_t mc2 = readReg(0x1E);   // spreading factor and CRC live here
  uint32_t wantFrf = (uint32_t)((915E6 * 524288.0) / 32E6);  // expected number for 915 MHz
  if (abs((long)frf - (long)wantFrf) > 2) return "frequency register does not match 915 MHz";
  if ((mc2 >> 4) != 9) return "spreading factor is not 9";
  if ((mc1 >> 4) != 7) return "bandwidth is not 125 kHz";
  if (((mc1 >> 1) & 7) != 1) return "coding rate is not 4/5";
  if (!(mc2 & 0x04)) return "CRC is not enabled";
  return "";
}

// Stuck here on purpose when startup fails, so I notice it
void initFailed(const String& why) {
  while (true) {
    Serial.print("# INIT FAILED: ");
    Serial.println(why);
    blinkLed(5);
    delay(500);
  }
}

// ---------- helpers ----------

// Gets one comma-separated field from a line
String getField(const String& s, int idx) {
  int start = 0;
  for (int i = 0; i < idx; i++) {
    start = s.indexOf(',', start);
    if (start < 0) return "";
    start++;
  }
  int end = s.indexOf(',', start);
  if (end < 0) end = s.length();
  return s.substring(start, end);
}

// Counts a character in a line. I use it to count commas to check the line format.
int countChar(const String& s, char ch) {
  int n = 0;
  for (unsigned i = 0; i < s.length(); i++) if (s[i] == ch) n++;
  return n;
}

// True only if every character is normal readable text.
// This blocks the garbage packets (like the zeros) from reaching the bridge.
bool isPrintable(const String& s) {
  for (unsigned i = 0; i < s.length(); i++) {
    if (s[i] < 32 || s[i] > 126) return false;
  }
  return true;
}

// Sends a D line from the PC out over the air to the hiker
void sendDownlink(const String& line) {
  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();          // transmit
  LoRa.receive();            // go back to listening for uplinks
  Serial.print("# downlink sent: ");
  Serial.println(line);      // bridge can see what was sent
}

// Checks a line from the PC. Only a proper D line is allowed to go to the hiker.
void processSerialLine(String line) {
  line.trim();               // remove spaces and newline leftovers
  if (line.length() == 0) return;
  // expected from the bridge: D,node,seq,TYPE (3 commas and a TYPE that is not empty)
  if (line.startsWith("D,") && countChar(line, ',') == 3 && getField(line, 3).length() > 0) {
    sendDownlink(line);
  } else {
    Serial.println("# serial line ignored (not a valid D line)");
  }
}

// Reads characters from the PC (USB) and builds a full line before using it
void handleSerialInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;                 // ignore Windows carriage return
    if (c == '\n') {                         // end of line, so the line is complete
      processSerialLine(serialLine);
      serialLine = "";                       // start fresh for the next line
    } else if (serialLine.length() < 80) {
      serialLine += c;                       // keep collecting
    } else {
      serialLine = "";                       // line is too long, so I drop it
    }
  }
}

// Checks if a LoRa packet arrived and prints it for the bridge
void handleRadio() {
  int size = LoRa.parsePacket();
  if (!size) return;                         // nothing arrived, leave

  String p = "";
  while (LoRa.available()) p += (char)LoRa.read();  // read the whole packet
  int rssi = LoRa.packetRssi();              // signal strength
  float snr = LoRa.packetSnr();              // signal quality vs noise

  // a valid uplink starts with U, ends with DIRECT, has 9 commas and is readable text
  bool ok = p.startsWith("U,") && p.endsWith(",DIRECT") &&
            countChar(p, ',') == 9 && isPrintable(p);
  if (ok) {
    // add rssi and snr at the end, because the bridge expects them from the base
    Serial.print(p);
    Serial.print(",");
    Serial.print(rssi);
    Serial.print(",");
    Serial.println(snr, 1);
  } else {
    // I print the length and rssi so I can see if junk is coming from a bad wire
    Serial.printf("# bad packet ignored (len=%d rssi=%d)\n", size, rssi);
  }
}

// Prints "still alive" every few seconds and also checks the chip is still answering
void heartbeat() {
  if (HEARTBEAT_MS == 0) return;                      // turned off
  if (millis() - lastBeat < HEARTBEAT_MS) return;     // not time yet
  lastBeat = millis();
  String bad = checkVersion();
  if (bad.length()) {
    Serial.print("# RADIO LOST: ");
    Serial.println(bad);
  } else {
    Serial.println("# listening... radio OK");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(1000);                 // wait for the serial monitor

  // Reset the chip by hand so it always starts clean
  pinMode(LORA_SS, OUTPUT);
  digitalWrite(LORA_SS, HIGH);
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(10);
  digitalWrite(LORA_RST, HIGH);
  delay(20);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

  // Step 1: is the chip answering?
  String bad = checkVersion();
  if (bad.length()) initFailed(bad);

  // Step 2: start the LoRa library
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(LORA_FREQ)) initFailed("LoRa.begin() returned false");

  // Same settings as the hiker. If one is different, they can't hear each other.
  LoRa.setTxPower(TX_POWER);
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();

  // Step 3: check the settings really reached the chip
  bad = checkSettings();
  if (bad.length()) initFailed(bad);

  LoRa.receive();              // start listening
  Serial.println("# INIT OK (version 0x12, 915 MHz, SF9, 125 kHz, CR 4/5, CRC on)");
  Serial.println("# BASE ready");   // the bridge looks for this text, so I keep it the same
}

void loop() {
  handleRadio();         // 1) any radio packets from the hiker?
  handleSerialInput();   // 2) any D line from the PC to send to the hiker?
  heartbeat();           // 3) say "still alive" and check the chip
}