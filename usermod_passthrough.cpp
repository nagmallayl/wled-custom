#include "wled.h"

#include <WiFi.h>
#include "esp_now.h"
#include "esp_wifi.h"

// ======================================================
// WLED ESP-NOW RECEIVER TEST V1
//
// WLED 16.0.0
// ESP32 Classic / GL-C-016WL-D
//
// Receives:
// 38 WS2811 IC
// 114 decoded color bytes
// 128-byte ESP-NOW packet
//
// CURRENT TEST:
// ESP-NOW RECEIVE ONLY
//
// NO LED OUTPUT FROM RECEIVED DATA YET.
// ======================================================


// ======================================================
// FRAME CONFIG
// ======================================================

#define EXPECTED_ICS 38

#define FRAME_BYTES \
  (EXPECTED_ICS * 3)


// ======================================================
// PACKET FORMAT
//
// MUST MATCH TRANSMITTER EXACTLY
// ======================================================

#define PACKET_MAGIC   0x2811
#define PACKET_VERSION 1


struct __attribute__((packed))
LedPacket
{
  uint16_t magic;

  uint8_t version;

  uint8_t icCount;

  uint32_t frameId;

  uint32_t txMicros;

  uint8_t data[
    FRAME_BYTES
  ];

  uint16_t crc;
};


// ======================================================
// QUEUE
//
// Callback receives packets quickly.
// Actual validation/printing happens in WLED loop.
//
// 8 packets × 128 bytes ≈ 1 KB.
// ======================================================

#define RX_QUEUE_LENGTH 8


struct RxQueueItem
{
  LedPacket packet;

  uint8_t senderMac[6];
};


static QueueHandle_t rxQueue =
  nullptr;


// ======================================================
// STATISTICS
// ======================================================

// Every ESP-NOW callback received.
static volatile uint32_t packetsReceived =
  0;


// Wrong packet length.
static volatile uint32_t badLength =
  0;


// Queue was full.
static volatile uint32_t queueDrops =
  0;


// Packets processed from queue.
static uint32_t packetsProcessed =
  0;


// Valid packets.
static uint32_t packetsGood =
  0;


// Validation errors.
static uint32_t badMagic =
  0;

static uint32_t badVersion =
  0;

static uint32_t badIcCount =
  0;

static uint32_t badCRC =
  0;


// Sequence diagnostics.
static uint32_t lastFrameId =
  0;

static uint32_t lostFrames =
  0;

static uint32_t duplicateFrames =
  0;

static uint32_t outOfOrderFrames =
  0;


// Last valid frame.
static uint8_t lastGoodFrame[
  FRAME_BYTES
];

static uint32_t lastGoodFrameId =
  0;

static uint32_t lastTxMicros =
  0;


// Sender information.
static uint8_t lastSenderMac[6] =
{
  0, 0, 0, 0, 0, 0
};


// Timing.
static uint32_t lastPacketMillis =
  0;

static uint32_t lastStatusMillis =
  0;

static uint32_t lastFramePrintMillis =
  0;


// ESP-NOW state.
static bool espNowReady =
  false;


// ======================================================
// CRC16 / CCITT
//
// MUST MATCH TRANSMITTER.
// ======================================================

static uint16_t calculateCRC16(
  const uint8_t *data,
  size_t length
)
{
  uint16_t crc =
    0xFFFF;


  for (
    size_t i = 0;
    i < length;
    i++
  )
  {
    crc ^=
      (uint16_t)data[i] << 8;


    for (
      uint8_t bit = 0;
      bit < 8;
      bit++
    )
    {
      if (
        crc & 0x8000
      )
      {
        crc =
          (crc << 1) ^
          0x1021;
      }
      else
      {
        crc <<=
          1;
      }
    }
  }


  return crc;
}


// ======================================================
// ESP-NOW RECEIVE CALLBACK
//
// Keep this callback very short.
//
// Arduino-ESP32 2.x / IDF 4.4 callback signature.
// ======================================================

static void onEspNowReceive(
  const uint8_t *macAddress,
  const uint8_t *incomingData,
  int len
)
{
  packetsReceived++;


  if (
    len != sizeof(LedPacket)
  )
  {
    badLength++;

    return;
  }


  if (
    rxQueue == nullptr
  )
  {
    queueDrops++;

    return;
  }


  RxQueueItem item;


  memcpy(
    &item.packet,
    incomingData,
    sizeof(LedPacket)
  );


  memcpy(
    item.senderMac,
    macAddress,
    6
  );


  /*
   * No blocking inside Wi-Fi callback.
   */

  BaseType_t result =
    xQueueSend(
      rxQueue,
      &item,
      0
    );


  if (
    result != pdTRUE
  )
  {
    queueDrops++;
  }
}


// ======================================================
// SETUP ESP-NOW
// ======================================================

static bool setupEspNowReceiver()
{
  Serial.println();

  Serial.println(
    "Starting ESP-NOW receiver..."
  );


  // ----------------------------------------------------
  // Create packet queue
  // ----------------------------------------------------

  rxQueue =
    xQueueCreate(
      RX_QUEUE_LENGTH,
      sizeof(RxQueueItem)
    );


  if (
    rxQueue == nullptr
  )
  {
    Serial.println(
      "RX QUEUE CREATE FAILED"
    );

    return false;
  }


  // ----------------------------------------------------
  // Initialize ESP-NOW
  // ----------------------------------------------------

  esp_err_t err =
    esp_now_init();


  if (
    err != ESP_OK
  )
  {
    Serial.print(
      "ESP-NOW INIT ERROR: "
    );

    Serial.println(
      esp_err_to_name(err)
    );

    return false;
  }


  // ----------------------------------------------------
  // Register RX callback
  // ----------------------------------------------------

  err =
    esp_now_register_recv_cb(
      onEspNowReceive
    );


  if (
    err != ESP_OK
  )
  {
    Serial.print(
      "ESP-NOW RX CALLBACK ERROR: "
    );

    Serial.println(
      esp_err_to_name(err)
    );

    return false;
  }


  espNowReady =
    true;


  Serial.println(
    "ESP-NOW RECEIVER READY"
  );


  return true;
}


// ======================================================
// SEQUENCE CHECK
// ======================================================

static void processFrameSequence(
  uint32_t frameId
)
{
  // First packet.
  if (
    lastFrameId == 0
  )
  {
    lastFrameId =
      frameId;

    return;
  }


  // Normal next packet.
  if (
    frameId ==
    lastFrameId + 1
  )
  {
    lastFrameId =
      frameId;

    return;
  }


  // Duplicate.
  if (
    frameId ==
    lastFrameId
  )
  {
    duplicateFrames++;

    return;
  }


  // Newer packet with gap.
  if (
    frameId >
    lastFrameId + 1
  )
  {
    lostFrames +=
      frameId -
      lastFrameId -
      1;


    lastFrameId =
      frameId;

    return;
  }


  // Older packet.
  outOfOrderFrames++;
}


// ======================================================
// PROCESS ONE PACKET
// ======================================================

static void processReceivedPacket(
  const RxQueueItem &item
)
{
  packetsProcessed++;


  const LedPacket &packet =
    item.packet;


  // ====================================================
  // MAGIC
  // ====================================================

  if (
    packet.magic !=
    PACKET_MAGIC
  )
  {
    badMagic++;

    return;
  }


  // ====================================================
  // VERSION
  // ====================================================

  if (
    packet.version !=
    PACKET_VERSION
  )
  {
    badVersion++;

    return;
  }


  // ====================================================
  // IC COUNT
  // ====================================================

  if (
    packet.icCount !=
    EXPECTED_ICS
  )
  {
    badIcCount++;

    return;
  }


  // ====================================================
  // CRC
  // ====================================================

  uint16_t calculatedCRC =
    calculateCRC16(
      (const uint8_t *)&packet,
      sizeof(LedPacket) -
      sizeof(packet.crc)
    );


  if (
    calculatedCRC !=
    packet.crc
  )
  {
    badCRC++;

    return;
  }


  // ====================================================
  // VALID FRAME
  // ====================================================

  packetsGood++;


  processFrameSequence(
    packet.frameId
  );


  memcpy(
    lastGoodFrame,
    packet.data,
    FRAME_BYTES
  );


  memcpy(
    lastSenderMac,
    item.senderMac,
    6
  );


  lastGoodFrameId =
    packet.frameId;


  lastTxMicros =
    packet.txMicros;


  lastPacketMillis =
    millis();
}


// ======================================================
// PROCESS QUEUE
// ======================================================

static void processRxQueue()
{
  if (
    rxQueue == nullptr
  )
  {
    return;
  }


  RxQueueItem item;


  /*
   * Process all packets currently waiting,
   * but never block WLED.
   */

  while (
    xQueueReceive(
      rxQueue,
      &item,
      0
    ) == pdTRUE
  )
  {
    processReceivedPacket(
      item
    );
  }
}


// ======================================================
// PRINT MAC
// ======================================================

static void printMac(
  const uint8_t *mac
)
{
  for (
    uint8_t i = 0;
    i < 6;
    i++
  )
  {
    if (
      i > 0
    )
    {
      Serial.print(
        ":"
      );
    }


    if (
      mac[i] < 16
    )
    {
      Serial.print(
        "0"
      );
    }


    Serial.print(
      mac[i],
      HEX
    );
  }
}


// ======================================================
// PRINT ONE IC
// ======================================================

static void printOneIC(
  uint8_t index
)
{
  uint16_t p =
    index * 3;


  Serial.print(
    "IC"
  );


  Serial.print(
    index + 1
  );


  Serial.print(
    ": G="
  );


  Serial.print(
    lastGoodFrame[
      p + 0
    ]
  );


  Serial.print(
    " R="
  );


  Serial.print(
    lastGoodFrame[
      p + 1
    ]
  );


  Serial.print(
    " B="
  );


  Serial.println(
    lastGoodFrame[
      p + 2
    ]
  );
}


// ======================================================
// PRINT FRAME SAMPLE
// ======================================================

static void printFrameSample()
{
  if (
    packetsGood == 0
  )
  {
    return;
  }


  Serial.println();

  Serial.println(
    "========== ESP-NOW RX FRAME =========="
  );


  Serial.print(
    "Frame ID: "
  );


  Serial.println(
    lastGoodFrameId
  );


  Serial.print(
    "Sender: "
  );


  printMac(
    lastSenderMac
  );


  Serial.println();


  Serial.print(
    "TX micros: "
  );


  Serial.println(
    lastTxMicros
  );


  printOneIC(0);
  printOneIC(1);
  printOneIC(2);


  Serial.println(
    "..."
  );


  printOneIC(
    EXPECTED_ICS - 3
  );


  printOneIC(
    EXPECTED_ICS - 2
  );


  printOneIC(
    EXPECTED_ICS - 1
  );


  Serial.println(
    "======================================"
  );
}


// ======================================================
// PRINT STATUS
// ======================================================

static void printStatus()
{
  Serial.println();


  Serial.print(
    "ESP RX: "
  );


  Serial.print(
    packetsReceived
  );


  Serial.print(
    "  Processed: "
  );


  Serial.print(
    packetsProcessed
  );


  Serial.print(
    "  Good: "
  );


  Serial.print(
    packetsGood
  );


  Serial.print(
    "  BadLen: "
  );


  Serial.print(
    badLength
  );


  Serial.print(
    "  QueueDrop: "
  );


  Serial.print(
    queueDrops
  );


  Serial.println();


  Serial.print(
    "CRC Bad: "
  );


  Serial.print(
    badCRC
  );


  Serial.print(
    "  Magic Bad: "
  );


  Serial.print(
    badMagic
  );


  Serial.print(
    "  Version Bad: "
  );


  Serial.print(
    badVersion
  );


  Serial.print(
    "  IC Bad: "
  );


  Serial.println(
    badIcCount
  );


  Serial.print(
    "Last Frame: "
  );


  Serial.print(
    lastGoodFrameId
  );


  Serial.print(
    "  Lost: "
  );


  Serial.print(
    lostFrames
  );


  Serial.print(
    "  Duplicate: "
  );


  Serial.print(
    duplicateFrames
  );


  Serial.print(
    "  OutOfOrder: "
  );


  Serial.println(
    outOfOrderFrames
  );


  Serial.print(
    "WiFi: "
  );


  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    Serial.print(
      "CONNECTED"
    );


    Serial.print(
      " RSSI="
    );


    Serial.print(
      WiFi.RSSI()
    );


    Serial.print(
      "dBm"
    );


    uint8_t primaryChannel =
      0;


    wifi_second_chan_t secondChannel =
      WIFI_SECOND_CHAN_NONE;


    esp_wifi_get_channel(
      &primaryChannel,
      &secondChannel
    );


    Serial.print(
      " CH="
    );


    Serial.print(
      primaryChannel
    );
  }
  else
  {
    Serial.print(
      "DISCONNECTED"
    );
  }


  Serial.print(
    "  ESP-NOW: "
  );


  Serial.println(
    espNowReady ?
    "READY" :
    "FAILED"
  );
}


// ======================================================
// USERMOD
// ======================================================

class EspNowReceiverUsermod :
  public Usermod
{

public:

  // ====================================================
  // SETUP
  // ====================================================

  void setup() override
  {
    memset(
      lastGoodFrame,
      0,
      sizeof(lastGoodFrame)
    );


    Serial.println();

    Serial.println(
      "================================"
    );


    Serial.println(
      "WLED ESP-NOW RECEIVER TEST V1"
    );


    Serial.println(
      "WLED 16.0.0"
    );


    Serial.println(
      "ESP32 Classic"
    );


    Serial.println(
      "================================"
    );


    Serial.print(
      "Expected ICs: "
    );


    Serial.println(
      EXPECTED_ICS
    );


    Serial.print(
      "Frame bytes: "
    );


    Serial.println(
      FRAME_BYTES
    );


    Serial.print(
      "ESP-NOW packet bytes: "
    );


    Serial.println(
      sizeof(LedPacket)
    );


    Serial.println(
      "LED output from ESP-NOW: DISABLED"
    );


    Serial.println(
      "CRC validation: ENABLED"
    );


    Serial.println(
      "Sequence tracking: ENABLED"
    );


    Serial.println(
      "================================"
    );


    /*
     * WLED itself handles Wi-Fi connection.
     *
     * Do not call WiFi.begin() here.
     */


    if (
      setupEspNowReceiver()
    )
    {
      Serial.println(
        "Receiver setup complete"
      );
    }
    else
    {
      Serial.println(
        "Receiver setup FAILED"
      );
    }


    Serial.println(
      "================================"
    );
  }


  // ====================================================
  // LOOP
  // ====================================================

  void loop() override
  {
    processRxQueue();


    uint32_t now =
      millis();


    // --------------------------------------------------
    // Print frame sample every 5 seconds.
    // --------------------------------------------------

    if (
      packetsGood > 0 &&
      now -
      lastFramePrintMillis >=
      5000
    )
    {
      lastFramePrintMillis =
        now;


      printFrameSample();
    }


    // --------------------------------------------------
    // Status every 2 seconds.
    // --------------------------------------------------

    if (
      now -
      lastStatusMillis >=
      2000
    )
    {
      lastStatusMillis =
        now;


      printStatus();
    }
  }


  // ====================================================
  // WLED INFO
  // ====================================================

  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "ESP-NOW Receiver"
      );


    info["ready"] =
      espNowReady;


    info["rx"] =
      packetsReceived;


    info["good"] =
      packetsGood;


    info["crc_bad"] =
      badCRC;


    info["lost"] =
      lostFrames;


    info["queue_drop"] =
      queueDrops;


    info["frame"] =
      lastGoodFrameId;
  }


  uint16_t getId() override
  {
    return 0x5054;
  }
};


// ======================================================
// REGISTER
// ======================================================

static EspNowReceiverUsermod
espNowReceiverUsermod;


REGISTER_USERMOD(
  espNowReceiverUsermod
);
