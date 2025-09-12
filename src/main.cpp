// CONTROL / SENDER: sends 'P' every 1s and prints any reply
#include <Arduino.h>
const int RX2_PIN = 16;  // ESP32 RX2  <- E32 TXD
const int TX2_PIN = 17;  // ESP32 TX2  -> E32 RXD

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.println("Sender up. Sending 'P' every 1s...");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();
    Serial2.write('P');                 // send one byte
    Serial.println("Sent: 'P'");
  }

  // If the other side echoes or replies, print it
  while (Serial2.available()) {
    char c = Serial2.read();
    Serial.print("Got reply: 0x");
    if ((uint8_t)c < 16) Serial.print('0');
    Serial.print((uint8_t)c, HEX);
    Serial.print(" '");
    Serial.print((c >= 32 && c <= 126) ? c : '.');
    Serial.println("'");
  }
}
