#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- Pins (per your wiring) ---
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_RXEN 32
#define LORA_TXEN 14

// Helpers to steer the RF front-end
static inline void radioListenPath() { digitalWrite(LORA_TXEN, LOW);  digitalWrite(LORA_RXEN, HIGH); }
static inline void radioTxPath()     { digitalWrite(LORA_RXEN, LOW);  digitalWrite(LORA_TXEN, HIGH); }

bool loraInit() {
  pinMode(LORA_RXEN, OUTPUT);
  pinMode(LORA_TXEN, OUTPUT);
  radioListenPath();

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  // Try a few times so you don't have to power-cycle
  for (int i = 0; i < 8; i++) {
    if (LoRa.begin(915E6)) {
      LoRa.setSyncWord(0x34);
      LoRa.enableCrc();
      LoRa.setSpreadingFactor(9);         // robust-ish
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.setTxPower(17, PA_OUTPUT_PA_BOOST_PIN); // safe drive level
      LoRa.receive();                     // start in RX
      radioListenPath();
      return true;
    }
    delay(400);
  }
  return false;
}

uint32_t seq = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!loraInit()) {
    Serial.println("LoRa INIT FAILED — check 3V3/CS/RST/DIO0/RXEN/TXEN/antenna.");
    while (1) delay(1000);
  }
  Serial.println("TX READY — heartbeating. Waiting for PONGs...");
}

void loop() {
  // Send PING
  char msg[24];
  snprintf(msg, sizeof(msg), "PING %lu", (unsigned long)seq);
  radioTxPath();
  LoRa.idle();                 // leave RX mode cleanly
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();            // blocking send
  radioListenPath();
  LoRa.receive();

  unsigned long t0 = millis();
  bool gotPong = false;

  // Wait up to 1200 ms for echo
  while (millis() - t0 < 1200) {
    int pkt = LoRa.parsePacket();
    if (pkt) {
      String rx;
      while (LoRa.available()) rx += (char)LoRa.read();
      if (rx.startsWith("PONG ")) {
        unsigned long rtt = millis() - t0;
        Serial.print("LINK OK  ");
        Serial.print(rx);
        Serial.print("  RSSI=");
        Serial.print(LoRa.packetRssi());
        Serial.print("  SNR=");
        Serial.println(LoRa.packetSnr(), 1);
        Serial.print("RTT: ");
        Serial.print(rtt);
        Serial.println(" ms");
        gotPong = true;
        break;
      }
    }
  }

  if (!gotPong) {
    Serial.println("No PONG (timeout).");
  }

  seq++;
  delay(1000);
}
