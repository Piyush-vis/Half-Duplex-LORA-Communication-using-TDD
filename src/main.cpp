/*
 * =============================================================
 *  Half-Duplex LoRa Transceiver with TDD
 *  Non-blocking ACK, Message Queue, CRC16 Integrity Check
 * =============================================================
 *  Hardware : ESP32 + AI-Thinker LoRa Ra-02 (SX1278, 433 MHz)
 *  Library  : sandeepmistry/LoRa @ ^0.8.0
 *  Pins     : SS=5, RST=14, DIO0=2
 * =============================================================
 *
 *  Packet format (pipe-delimited):
 *    Data : #LM|MSG|<id>|<payload>|<crc16_hex>
 *    ACK  : #LM|ACK|<id>|<crc16_hex>
 *
 *  CRC16 (CCITT-FALSE) is computed over the body between the
 *  signature and the CRC field itself, providing application-
 *  layer error detection on top of LoRa's built-in FEC.
 *
 *  If CRC fails on the receiver, the packet is silently dropped
 *  (no ACK sent), which forces the sender to retransmit.
 * =============================================================
 */

#include <Arduino.h>
#include <LoRa.h>

// ─── Hardware Pin Mapping (AI-Thinker Ra-02 → ESP32) ────────
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2

// ─── Radio Parameters ───────────────────────────────────────
#define LORA_FREQ         433E6   // 433 MHz band
#define LORA_TX_POWER     2       // Low power (2 dBm) for bench testing
#define LORA_SF           7       // Spreading Factor 7

// ─── Protocol Constants ─────────────────────────────────────
#define SIGNATURE         "#LM"   // System signature prefix
#define ACK_TIMEOUT_MS    2000    // Wait 2 s for ACK before retry
#define MAX_RETRIES       5       // 5 attempts before giving up
#define MAX_QUEUE         5       // Max outgoing messages waiting for ACK
#define DEDUP_HISTORY     16      // Remember last N received msg IDs for dedup
#define SERIAL_BUF_SIZE   200     // Max characters in one typed message

// ─── Message Queue ──────────────────────────────────────────
struct PendingMsg {
  String   payload;
  uint8_t  msgId;
  uint8_t  retriesLeft;
  unsigned long lastSendTime;
  bool     active;
};

PendingMsg msgQueue[MAX_QUEUE];
uint8_t    nextMsgId = 0;   // Rolling 0-255 counter

// ─── Duplicate Detection Ring Buffer ────────────────────────
uint8_t dedupHistory[DEDUP_HISTORY];
uint8_t dedupIndex = 0;
bool    dedupFull  = false;   // Has the ring buffer wrapped around?

// ─── Serial Input Buffer ────────────────────────────────────
String serialInputBuf = "";

// ═════════════════════════════════════════════════════════════
//  CRC-16 / CCITT-FALSE  (poly 0x1021, init 0xFFFF)
//  Provides application-layer integrity verification.
// ═════════════════════════════════════════════════════════════
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Convenience: compute CRC on a String
uint16_t crc16_str(const String &s) {
  return crc16_ccitt((const uint8_t *)s.c_str(), s.length());
}

// ═════════════════════════════════════════════════════════════
//  Packet Builder Helpers
// ═════════════════════════════════════════════════════════════

// Build a DATA packet string
String buildDataPacket(uint8_t id, const String &payload) {
  // Body over which CRC is calculated: MSG|<id>|<payload>
  String body = "MSG|" + String(id) + "|" + payload;
  uint16_t crc = crc16_str(body);
  char crcHex[5];
  sprintf(crcHex, "%04X", crc);
  return String(SIGNATURE) + "|" + body + "|" + String(crcHex);
}

// Build an ACK packet string
String buildAckPacket(uint8_t id) {
  String body = "ACK|" + String(id);
  uint16_t crc = crc16_str(body);
  char crcHex[5];
  sprintf(crcHex, "%04X", crc);
  return String(SIGNATURE) + "|" + body + "|" + String(crcHex);
}

// ═════════════════════════════════════════════════════════════
//  Transmit a raw LoRa packet (switches radio to TX then back)
// ═════════════════════════════════════════════════════════════
void loraSend(const String &packet) {
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
  LoRa.receive();   // Switch back to RX immediately (half-duplex)
}

// ═════════════════════════════════════════════════════════════
//  Duplicate Detection
// ═════════════════════════════════════════════════════════════
bool isDuplicate(uint8_t id) {
  uint8_t limit = dedupFull ? DEDUP_HISTORY : dedupIndex;
  for (uint8_t i = 0; i < limit; i++) {
    if (dedupHistory[i] == id) return true;
  }
  return false;
}

void recordMsgId(uint8_t id) {
  dedupHistory[dedupIndex] = id;
  dedupIndex = (dedupIndex + 1) % DEDUP_HISTORY;
  if (dedupIndex == 0) dedupFull = true;
}

// ═════════════════════════════════════════════════════════════
//  Queue Management
// ═════════════════════════════════════════════════════════════

// Find a free slot; returns index or -1 if buffer full
int findFreeSlot() {
  for (int i = 0; i < MAX_QUEUE; i++) {
    if (!msgQueue[i].active) return i;
  }
  return -1;
}

// Enqueue a message, send immediately, and start ACK timer
void enqueueMessage(const String &payload) {
  int slot = findFreeSlot();
  if (slot == -1) {
    Serial.println("[!] Buffer full! Wait for ACKs or timeouts before sending more.");
    return;
  }

  uint8_t id = nextMsgId++;   // wraps 0-255 automatically (uint8_t)

  msgQueue[slot].payload      = payload;
  msgQueue[slot].msgId        = id;
  msgQueue[slot].retriesLeft  = MAX_RETRIES;
  msgQueue[slot].lastSendTime = millis();
  msgQueue[slot].active       = true;

  String pkt = buildDataPacket(id, payload);
  loraSend(pkt);

  Serial.println("[TX] Sent (id=" + String(id) + "): " + payload);
}

// ═════════════════════════════════════════════════════════════
//  Handle Incoming LoRa Packets
// ═════════════════════════════════════════════════════════════
void handleIncomingPacket(const String &raw, int rssi, float snr) {
  // ── 1. Check signature ─────────────────────────────────
  if (!raw.startsWith(String(SIGNATURE) + "|")) {
    // Not our system – silently ignore
    return;
  }

  // Strip signature: everything after "#LM|"
  String content = raw.substring(String(SIGNATURE).length() + 1);

  // ── 2. Extract and verify CRC ──────────────────────────
  int lastPipe = content.lastIndexOf('|');
  if (lastPipe == -1) return;   // Malformed

  String body   = content.substring(0, lastPipe);
  String crcStr = content.substring(lastPipe + 1);

  uint16_t receivedCrc = (uint16_t)strtoul(crcStr.c_str(), NULL, 16);
  uint16_t computedCrc = crc16_str(body);

  if (receivedCrc != computedCrc) {
    Serial.println("[!] CRC mismatch – packet corrupted, dropping (no ACK).");
    return;
  }

  // ── 3. Parse type and id ───────────────────────────────
  // body = "MSG|<id>|<payload>"  or  "ACK|<id>"
  int firstPipe  = body.indexOf('|');
  if (firstPipe == -1) return;

  String type = body.substring(0, firstPipe);
  String rest = body.substring(firstPipe + 1);

  if (type == "MSG") {
    // rest = "<id>|<payload>"
    int idPipe = rest.indexOf('|');
    if (idPipe == -1) return;

    uint8_t id     = (uint8_t)rest.substring(0, idPipe).toInt();
    String payload = rest.substring(idPipe + 1);

    // Always send ACK (original ACK may have been lost)
    String ack = buildAckPacket(id);
    loraSend(ack);

    if (!isDuplicate(id)) {
      recordMsgId(id);
      Serial.println("────────────────────────────────────");
      Serial.println("[RX] Message received (id=" + String(id) + ")");
      Serial.println("     >> " + payload);
      Serial.println("     RSSI: " + String(rssi) + " dBm | SNR: " + String(snr, 1) + " dB");
      Serial.println("     ACK sent back.");
      Serial.println("────────────────────────────────────");
    } else {
      Serial.println("[RX] Duplicate msg id=" + String(id) + " – ACK re-sent.");
    }

  } else if (type == "ACK") {
    uint8_t id = (uint8_t)rest.toInt();

    // Find matching pending message and clear it
    for (int i = 0; i < MAX_QUEUE; i++) {
      if (msgQueue[i].active && msgQueue[i].msgId == id) {
        msgQueue[i].active = false;
        Serial.println("[ACK] Message id=" + String(id) + " delivered!");
        Serial.println("      RSSI: " + String(rssi) + " dBm | SNR: " + String(snr, 1) + " dB");
        return;
      }
    }
    // ACK for unknown id – probably already timed out or duplicate ACK
  }
}

// ═════════════════════════════════════════════════════════════
//  Retry / Timeout Scanner
// ═════════════════════════════════════════════════════════════
void scanQueueForRetries() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_QUEUE; i++) {
    if (!msgQueue[i].active) continue;

    if (now - msgQueue[i].lastSendTime >= ACK_TIMEOUT_MS) {
      if (msgQueue[i].retriesLeft > 0) {
        msgQueue[i].retriesLeft--;
        msgQueue[i].lastSendTime = now;

        String pkt = buildDataPacket(msgQueue[i].msgId, msgQueue[i].payload);
        loraSend(pkt);

        Serial.println("[RETRY] id=" + String(msgQueue[i].msgId)
                       + " retries left: " + String(msgQueue[i].retriesLeft));
      } else {
        // All retries exhausted
        Serial.println("════════════════════════════════════");
        Serial.println("[FAIL] No device or node in the range!");
        Serial.println("       Message id=" + String(msgQueue[i].msgId)
                       + " could not be delivered:");
        Serial.println("       >> " + msgQueue[i].payload);
        Serial.println("════════════════════════════════════");
        msgQueue[i].active = false;
      }
    }
  }
}

// ═════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial);   // Wait for Serial to be ready

  Serial.println();
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   LoRa Half-Duplex Transceiver (TDD)    ║");
  Serial.println("║   SF=7  TxPwr=2dBm  Freq=433MHz         ║");
  Serial.println("╚══════════════════════════════════════════╝");

  // ── Initialise LoRa ────────────────────────────────────
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("[FATAL] LoRa module failed to start!");
    while (1);   // Halt
  }

  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.receive();   // Start in receive mode

  // Initialise queue
  for (int i = 0; i < MAX_QUEUE; i++) {
    msgQueue[i].active = false;
  }

  Serial.println("[OK] LoRa ready. Type a message and press Enter to send.");
  Serial.println();
}

// ═════════════════════════════════════════════════════════════
//  LOOP  (fully non-blocking)
// ═════════════════════════════════════════════════════════════
void loop() {

  // ── 1. Read Serial Input ───────────────────────────────
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      serialInputBuf.trim();
      if (serialInputBuf.length() > 0) {
        enqueueMessage(serialInputBuf);
        serialInputBuf = "";
      }
    } else {
      if (serialInputBuf.length() < SERIAL_BUF_SIZE) {
        serialInputBuf += ch;
      }
    }
  }

  // ── 2. Check for Incoming LoRa Packets ─────────────────
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    int   rssi = LoRa.packetRssi();
    float snr  = LoRa.packetSnr();
    handleIncomingPacket(incoming, rssi, snr);
  }

  // ── 3. Retry / Timeout Scanner ─────────────────────────
  scanQueueForRetries();
}
