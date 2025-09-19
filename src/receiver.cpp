//receiver code


#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// --- Pins (per your wiring) ---
#define LORA_CS   5
#define LORA_RST  27
#define LORA_DIO0 26
#define LORA_RXEN 32
#define LORA_TXEN 14

static inline void radioListenPath() { digitalWrite(LORA_TXEN, LOW);  digitalWrite(LORA_RXEN, HIGH); }
static inline void radioTxPath()     { digitalWrite(LORA_RXEN, LOW);  digitalWrite(LORA_TXEN, HIGH); }

bool loraInit() {
  pinMode(LORA_RXEN, OUTPUT);
  pinMode(LORA_TXEN, OUTPUT);
  radioListenPath();

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

  for (int i = 0; i < 8; i++) {
    if (LoRa.begin(915E6)) {
      LoRa.setSyncWord(0x34);
      LoRa.enableCrc();
      LoRa.setSpreadingFactor(9);
      LoRa.setSignalBandwidth(125E3);
      LoRa.setCodingRate4(5);
      LoRa.receive();          // continuous RX
      radioListenPath();
      return true;
    }
    delay(400);
  }
  return false;
}

void echoPong(const String& ping) {
  // Extract seq after "PING "
  String tail = ping.substring(5);
  String pong = "PONG " + tail;

  radioTxPath();
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(pong);
  LoRa.endPacket();
  radioListenPath();
  LoRa.receive();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!loraInit()) {
    Serial.println("LoRa INIT FAILED — check wiring/power/antenna.");
    while (1) delay(1000);
  }

  Serial.println("RX READY — listening for packets.");
  Serial.println("(Type a line and press Enter to transmit it once if you want to test TX.)");
}

void loop() {
  // LoRa receive path
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String data;
    while (LoRa.available()) data += (char)LoRa.read();

    Serial.print("RX: ");
    Serial.print(data);
    Serial.print(" | RSSI=");
    Serial.print(LoRa.packetRssi());
    Serial.print(" | SNR=");
    Serial.println(LoRa.packetSnr(), 1);

    if (data.startsWith("PING ")) {
      echoPong(data);
      Serial.println("   -> PONG echoed");
    }
  }

  // Optional: let you send manual test packets from the PC
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length()) {
      radioTxPath();
      LoRa.idle();
      LoRa.beginPacket();
      LoRa.print(line);
      LoRa.endPacket();
      radioListenPath();
      LoRa.receive();
      Serial.println(String("TX: ") + line);
    }
  }
}
