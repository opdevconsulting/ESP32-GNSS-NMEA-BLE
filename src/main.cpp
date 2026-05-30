#include <Arduino.h>
#include <NimBLEDevice.h>

// UART pins for ESP32-C3 (UART1)
#define GPS_RX 4   // ESP32-C3 RX <- GPS TX
#define GPS_TX 5   // ESP32-C3 TX -> GPS RX

#define GPS_BAUD 115200

// Nordic UART Service (NUS) UUIDs - common BLE serial standard
#define NUS_SERVICE_UUID   "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_TX_CHAR_UUID   "6e400003-b5a3-f393-e0a9-e50e24dcca9e" // Notify (device -> phone)
#define NUS_RX_CHAR_UUID   "6e400002-b5a3-f393-e0a9-e50e24dcca9e" // Write  (phone -> device)

// Lap Legend "Universal NMEA GPS via BLE" service: FFF0 / notify on FFF1
// (per Dauntless Devices, the convention many BLE serial modules now use)
#define LAPL_SERVICE_UUID  "0000fff0-0000-1000-8000-00805f9b34fb"
#define LAPL_CHAR_UUID     "0000fff1-0000-1000-8000-00805f9b34fb" // Notify (+ R/W for compat)
#define LAPL_WR_CHAR_UUID  "0000fff2-0000-1000-8000-00805f9b34fb" // Write (common BLE-SPP layout)

// Per-unit BLE name is built at boot from the chip MAC (LapLegendGPS-XXXXXX)
#define BLE_DEVICE_NAME_PREFIX "LapLegendGPS-"
char bleDeviceName[24];

HardwareSerial GPS_Serial(1);
NimBLECharacteristic *pNusTxChar = nullptr;
NimBLECharacteristic *pLaplChar = nullptr;
NimBLEServer *pServer = nullptr;
volatile bool deviceConnected = false;
volatile bool nusSubscribed = false;
volatile bool laplSubscribed = false;
volatile uint16_t currentMTU = 23;

// NMEA line buffer
#define NMEA_BUF_SIZE 128
char nmeaBuf[NMEA_BUF_SIZE];
uint16_t nmeaIdx = 0;
bool nmeaCapture = false;

// Stats
unsigned long bytesSent = 0;
unsigned long sentencesSent = 0;
unsigned long sentencesDropped = 0;
unsigned long lastStatPrint = 0;

// Delayed GPS config resend (cold start workaround)
bool gpsConfigResent = false;
unsigned long bootTime = 0;

// ---------------- BLE callbacks ----------------

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    deviceConnected = true;
    Serial.printf("BLE client connected (handle=%u)\n", desc->conn_handle);
    // Do NOT restart advertising here - that would allow multiple parallel connections.
    // Do NOT call updateConnParams here either - let the central negotiate.
  }

  // NimBLE-Arduino 1.4.x signature: 2 args, no `reason`
  void onDisconnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) {
    deviceConnected = false;
    nusSubscribed = false;
    laplSubscribed = false;
    currentMTU = 23;
    Serial.printf("BLE client disconnected (handle=%u)\n", desc->conn_handle);
    NimBLEDevice::startAdvertising();
  }

  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc) {
    currentMTU = MTU;
    Serial.printf("MTU negotiated: %u (payload: %u)\n", MTU, MTU - 3);
  }
};

class NusTxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc, uint16_t subValue) {
    nusSubscribed = (subValue & 0x01) != 0;
    Serial.printf("NUS CCCD: %s (val=%u)\n",
                  nusSubscribed ? "NOTIFY" : "off", subValue);
  }
  void onRead(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc) {
    Serial.println("NUS TX read");
  }
};

class NusRxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc) {
    std::string v = pChar->getValue();
    Serial.printf("NUS RX write (%u bytes)\n", (unsigned)v.size());
  }
};

class Hm10Callbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc, uint16_t subValue) {
    laplSubscribed = (subValue & 0x01) != 0;
    Serial.printf("FFF1 CCCD: %s (val=%u)\n",
                  laplSubscribed ? "NOTIFY" : "off", subValue);
  }
  void onRead(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc) {
    Serial.println("FFF1 read");
  }
  void onWrite(NimBLECharacteristic *pChar, ble_gap_conn_desc *desc) {
    std::string v = pChar->getValue();
    Serial.printf("FFF1 write (%u bytes)\n", (unsigned)v.size());
  }
};

// ---------------- NMEA processing ----------------

// Accept $G_RMC and $G_GGA from any talker (GP/GN/GL/GA/GB).
static bool wantSentence(const char *buf, uint16_t len) {
  if (len < 7 || buf[0] != '$' || buf[1] != 'G') return false;
  const char *t = buf + 3; // skip "$G?"
  return (t[0] == 'R' && t[1] == 'M' && t[2] == 'C') ||
         (t[0] == 'G' && t[1] == 'G' && t[2] == 'A');
}

static void bleSendNmea(const char *sentence, uint16_t len) {
  if (!deviceConnected || len == 0) return;
  if (!nusSubscribed && !laplSubscribed) return;

  uint16_t payload = currentMTU > 3 ? (currentMTU - 3) : 20;

  // Send to whichever services the client has subscribed to
  if (len <= payload) {
    if (nusSubscribed) {
      pNusTxChar->setValue((uint8_t *)sentence, len);
      pNusTxChar->notify();
    }
    if (laplSubscribed) {
      pLaplChar->setValue((uint8_t *)sentence, len);
      pLaplChar->notify();
    }
  } else {
    // Chunked fallback for tiny MTU
    uint16_t offset = 0;
    while (offset < len) {
      uint16_t chunk = min((uint16_t)(len - offset), payload);
      if (nusSubscribed) {
        pNusTxChar->setValue((uint8_t *)(sentence + offset), chunk);
        pNusTxChar->notify();
      }
      if (laplSubscribed) {
        pLaplChar->setValue((uint8_t *)(sentence + offset), chunk);
        pLaplChar->notify();
      }
      offset += chunk;
    }
  }
  bytesSent += len;
  sentencesSent++;
}

// ---------------- GPS configuration ----------------

// Compute and append UBX checksum (last 2 bytes of buf must be placeholder)
static void ubxChecksum(uint8_t *buf, size_t len) {
  uint8_t ckA = 0, ckB = 0;
  for (size_t i = 2; i < len - 2; i++) { ckA += buf[i]; ckB += ckA; }
  buf[len - 2] = ckA;
  buf[len - 1] = ckB;
}

static void sendSetRate10Hz() {
  uint8_t pkt[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x01, 0x00, 0x21, 0x30, // key: CFG-RATE-MEAS
    0x64, 0x00,             // value: 100ms (10 Hz)
    0x00, 0x00              // checksum placeholder
  };
  ubxChecksum(pkt, sizeof(pkt));
  GPS_Serial.write(pkt, sizeof(pkt));
  GPS_Serial.flush();
}

static void sendMsgFilter() {
  // Disable GLL, VTG, GSA, GSV on UART1; enable GGA, RMC
  uint8_t pkt[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x22, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0xCA, 0x00, 0x91, 0x20, 0x00, // GLL off
    0xB1, 0x00, 0x91, 0x20, 0x00, // VTG off
    0xC0, 0x00, 0x91, 0x20, 0x00, // GSA off
    0xC5, 0x00, 0x91, 0x20, 0x00, // GSV off
    0xBB, 0x00, 0x91, 0x20, 0x01, // GGA on (every meas)
    0xAC, 0x00, 0x91, 0x20, 0x01, // RMC on (every meas)
    0x00, 0x00                    // checksum placeholder
  };
  ubxChecksum(pkt, sizeof(pkt));
  GPS_Serial.write(pkt, sizeof(pkt));
  GPS_Serial.flush();
}

// ---------------- Setup ----------------

void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n--- C3-LAPL-452 NMEA BLE GPS (NimBLE) ---");

  // UBX-CFG-VALSET: Set UART1 baudrate to 115200
  uint8_t setBaud115200[] = {
    0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00,
    0x00, 0x01, 0x00, 0x00,
    0x01, 0x00, 0x52, 0x40,
    0x00, 0xC2, 0x01, 0x00,
    0xF3, 0xA5
  };

  // Try common baud rates so we land on 115200 regardless of prior state
  GPS_Serial.begin(38400, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(1000);
  Serial.println("Sending baud change at 38400...");
  GPS_Serial.write(setBaud115200, sizeof(setBaud115200));
  GPS_Serial.flush();
  delay(100);

  GPS_Serial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(50);
  Serial.println("Sending baud change at 9600...");
  GPS_Serial.write(setBaud115200, sizeof(setBaud115200));
  GPS_Serial.flush();
  delay(100);

  GPS_Serial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(50);
  Serial.println("Sending baud change at 115200...");
  GPS_Serial.write(setBaud115200, sizeof(setBaud115200));
  GPS_Serial.flush();
  delay(100);
  while (GPS_Serial.available()) GPS_Serial.read();

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(500);

  sendSetRate10Hz();
  delay(100);
  sendMsgFilter();
  delay(100);

  Serial.println("GPS configured: 115200 baud, 10Hz, GGA+RMC only");
  bootTime = millis();

  // -------- BLE (NimBLE) --------
  // Build a per-unit name from the chip's MAC so multiple devices don't collide
  uint64_t mac = ESP.getEfuseMac();
  snprintf(bleDeviceName, sizeof(bleDeviceName), "%s%02X%02X%02X",
           BLE_DEVICE_NAME_PREFIX,
           (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)mac);
  Serial.printf("BLE name: %s\n", bleDeviceName);

  NimBLEDevice::init(bleDeviceName);
  // Default TX power (+3 dBm) - reliable in-cabin link
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  // Request larger MTU; Android typically agrees to ~247
  NimBLEDevice::setMTU(247);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // ---- Nordic UART Service ----
  NimBLEService *pNusService = pServer->createService(NUS_SERVICE_UUID);
  pNusTxChar = pNusService->createCharacteristic(
                 NUS_TX_CHAR_UUID,
                 NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  pNusTxChar->setCallbacks(new NusTxCallbacks());
  NimBLECharacteristic *pNusRx = pNusService->createCharacteristic(
    NUS_RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pNusRx->setCallbacks(new NusRxCallbacks());
  pNusService->start();

  // ---- Lap Legend FFF0 service (Notify on FFF1, optional Write on FFF2) ----
  NimBLEService *pLaplService = pServer->createService(LAPL_SERVICE_UUID);
  pLaplChar = pLaplService->createCharacteristic(
                LAPL_CHAR_UUID,
                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY |
                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pLaplChar->setCallbacks(new Hm10Callbacks());
  NimBLECharacteristic *pLaplWr = pLaplService->createCharacteristic(
                LAPL_WR_CHAR_UUID,
                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pLaplWr->setCallbacks(new Hm10Callbacks());
  pLaplService->start();

  // ---- Advertising ----
  // Adv packet: flags + FFF0 16-bit UUID (this is what Lap Legend filters on)
  // Scan response: device name + NUS 128-bit UUID (diagnostic / nRF Connect)
  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(LAPL_SERVICE_UUID);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(bleDeviceName);
  scanResp.setCompleteServices(NimBLEUUID(NUS_SERVICE_UUID));
  pAdv->setScanResponseData(scanResp);

  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMaxPreferred(0x12);
  pAdv->start();

  Serial.println("BLE server started: FFF0 + NUS services advertised");
}

// ---------------- Loop ----------------

void loop() {
  // Drain UART, assemble NMEA sentences, filter, forward
  while (GPS_Serial.available()) {
    char c = GPS_Serial.read();

    if (c == '$') {
      nmeaCapture = true;
      nmeaIdx = 0;
      nmeaBuf[nmeaIdx++] = c;
    } else if (nmeaCapture) {
      if (nmeaIdx < NMEA_BUF_SIZE - 1) {
        nmeaBuf[nmeaIdx++] = c;
      } else {
        nmeaCapture = false;
        nmeaIdx = 0;
        sentencesDropped++;
      }
      if (c == '\n') {
        nmeaBuf[nmeaIdx] = '\0';
        if (wantSentence(nmeaBuf, nmeaIdx)) {
          bleSendNmea(nmeaBuf, nmeaIdx);
        }
        nmeaCapture = false;
        nmeaIdx = 0;
      }
    }
  }

  unsigned long now = millis();

  // Resend GPS config 5s after boot - covers cold-start race
  if (!gpsConfigResent && (now - bootTime >= 5000)) {
    gpsConfigResent = true;
    Serial.println("Resending GPS 10Hz + filter config...");
    sendSetRate10Hz();
    delay(50);
    sendMsgFilter();
  }

  // Periodic stats
  if (now - lastStatPrint >= 10000) {
    Serial.printf("BLE %s%s%s | MTU=%u | sent=%lu sentences=%lu drop=%lu\n",
                  deviceConnected ? "CONN" : "wait",
                  nusSubscribed ? "+NUS" : "",
                  laplSubscribed ? "+FFF1" : "",
                  currentMTU, bytesSent, sentencesSent, sentencesDropped);
    lastStatPrint = now;
  }
}
