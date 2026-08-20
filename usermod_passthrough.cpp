#include "wled.h"

#include <WiFi.h>
#include "esp_now.h"
#include "esp_wifi.h"

// ======================================================
// WLED ESP-NOW RECEIVER V3.1
//
// WLED 16.0.0
// ESP32 Classic / GL-C-016WL-D
//
// ESP-NOW RX
// 38 WS2811 IC
// 114 decoded bytes
//
// OUTPUT:
// WLED pixels 0..37
// GPIO16 / I2S
//
// Incoming packet bytes:
// G R B
//
// WLED output:
// Color Order in LED Preferences = BRG
//
// IMPORTANT:
// We pass RGB normally to WLED:
// R = inputR
// G = inputG
// B = inputB
//
// WLED's configured BRG color order handles
// the physical strip ordering.
// ======================================================


// ======================================================
// CONFIG
// ======================================================

#define EXPECTED_ICS 38

#define FRAME_BYTES \
  (EXPECTED_ICS * 3)

#define PACKET_MAGIC   0x2811
#define PACKET_VERSION 1

#define SIGNAL_TIMEOUT_MS 1500
#define REALTIME_LOCK_MS  1000

#define RX_QUEUE_LENGTH 8


// ======================================================
// PACKET
// ======================================================

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
// ======================================================

struct RxQueueItem
{
  LedPacket packet;

  uint8_t senderMac[6];
};


static QueueHandle_t rxQueue =
  nullptr;


// ======================================================
// LAST FRAME
// ======================================================

static uint8_t lastGoodFrame[
  FRAME_BYTES
];

static volatile bool newFrameReady =
  false;


// ======================================================
// STATE
// ======================================================

static bool espNowReady =
  false;

static bool realtimeActive =
  false;

static uint32_t lastInitAttemptMs =
  0;

static uint32_t lastPacketMillis =
  0;


// ======================================================
// STATISTICS
// ======================================================

static volatile uint32_t packetsReceived =
  0;

static volatile uint32_t badLength =
  0;

static volatile uint32_t queueDrops =
  0;


static uint32_t packetsProcessed =
  0;

static uint32_t packetsGood =
  0;

static uint32_t framesShown =
  0;


static uint32_t badMagic =
  0;

static uint32_t badVersion =
  0;

static uint32_t badIcCount =
  0;

static uint32_t badCRC =
  0;


static uint32_t lastFrameId =
  0;

static uint32_t lostFrames =
  0;

static uint32_t duplicateFrames =
  0;

static uint32_t outOfOrderFrames =
  0;


static uint32_t lastStatusMillis =
  0;


// ======================================================
// CRC16
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


  if (
    xQueueSend(
      rxQueue,
      &item,
      0
    ) != pdTRUE
  )
  {
    queueDrops++;
  }
}


// ======================================================
// CREATE RX QUEUE
// ======================================================

static bool createRxQueue()
{
  if (
    rxQueue != nullptr
  )
  {
    return true;
  }


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


  Serial.println(
    "RX QUEUE READY"
  );


  return true;
}


// ======================================================
// START ESP-NOW
// ======================================================

static bool startEspNow()
{
  if (
    espNowReady
  )
  {
    return true;
  }


  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    return false;
  }


  Serial.println();

  Serial.println(
    "Wi-Fi ready -> starting ESP-NOW..."
  );


  // ====================================================
  // CHECK WIFI CHANNEL
  // ====================================================

  uint8_t channel =
    0;


  wifi_second_chan_t secondary =
    WIFI_SECOND_CHAN_NONE;


  esp_wifi_get_channel(
    &channel,
    &secondary
  );


  Serial.print(
    "Wi-Fi Channel: "
  );


  Serial.println(
    channel
  );


  // ====================================================
  // INIT ESP-NOW
  // ====================================================

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


  // ====================================================
  // REGISTER RECEIVE CALLBACK
  // ====================================================

  err =
    esp_now_register_recv_cb(
      onEspNowReceive
    );


  if (
    err != ESP_OK
  )
  {
    Serial.print(
      "ESP-NOW CALLBACK ERROR: "
    );


    Serial.println(
      esp_err_to_name(err)
    );


    esp_now_deinit();


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
// FRAME SEQUENCE
// ======================================================

static void processFrameSequence(
  uint32_t frameId
)
{
  // First packet
  if (
    lastFrameId == 0
  )
  {
    lastFrameId =
      frameId;


    return;
  }


  // Expected next packet
  if (
    frameId ==
    lastFrameId + 1
  )
  {
    lastFrameId =
      frameId;


    return;
  }


  // Duplicate
  if (
    frameId ==
    lastFrameId
  )
  {
    duplicateFrames++;


    return;
  }


  // Newer packet with gap
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


  // Older frame
  outOfOrderFrames++;
}


// ======================================================
// PROCESS RECEIVED PACKET
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

  uint16_t crc =
    calculateCRC16(
      (const uint8_t *)&packet,
      sizeof(LedPacket) -
      sizeof(packet.crc)
    );


  if (
    crc !=
    packet.crc
  )
  {
    badCRC++;


    return;
  }


  // ====================================================
  // GOOD PACKET
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


  newFrameReady =
    true;


  lastPacketMillis =
    millis();
}


// ======================================================
// PROCESS RX QUEUE
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
// APPLY FRAME TO WLED
// ======================================================

static void applyFrameToWLED()
{
  if (
    !newFrameReady
  )
  {
    return;
  }


  // ====================================================
  // REALTIME LOCK
  // ====================================================

  realtimeLock(
    REALTIME_LOCK_MS,
    REALTIME_MODE_UDP
  );


  realtimeActive =
    true;


  // ====================================================
  // COLOR CONVERSION
  //
  // Incoming packet:
  //
  // byte0 = G
  // byte1 = R
  // byte2 = B
  //
  // WLED setPixelColor() expects logical:
  //
  // R, G, B
  //
  // WLED LED Preferences itself is configured:
  //
  // Color Order = BRG
  //
  // Therefore DO NOT manually rotate channels here.
  // ====================================================

  for (
    uint16_t i = 0;
    i < EXPECTED_ICS;
    i++
  )
  {
    uint16_t p =
      i * 3;


    uint8_t inputG =
      lastGoodFrame[
        p + 0
      ];


    uint8_t inputR =
      lastGoodFrame[
        p + 1
      ];


    uint8_t inputB =
      lastGoodFrame[
        p + 2
      ];


    // ==================================================
    // FIXED COLOR MAPPING
    //
    // Logical RGB into WLED.
    //
    // WLED then applies physical BRG order.
    // ==================================================

    strip.setPixelColor(
      i,

      inputR,

      inputG,

      inputB
    );
  }


  // ====================================================
  // OUTPUT
  // ====================================================

  strip.show();


  framesShown++;


  newFrameReady =
    false;
}


// ======================================================
// REALTIME TIMEOUT
// ======================================================

static void updateRealtimeTimeout()
{
  if (
    !realtimeActive
  )
  {
    return;
  }


  if (
    millis() -
    lastPacketMillis >
    SIGNAL_TIMEOUT_MS
  )
  {
    realtimeActive =
      false;


    newFrameReady =
      false;


    exitRealtime();


    Serial.println(
      "ESP-NOW signal lost -> WLED effects resumed"
    );
  }
}


// ======================================================
// PRINT STATUS
// ======================================================

static void printStatus()
{
  Serial.println();


  Serial.print(
    "RX: "
  );


  Serial.print(
    packetsReceived
  );


  Serial.print(
    "  Good: "
  );


  Serial.print(
    packetsGood
  );


  Serial.print(
    "  Shown: "
  );


  Serial.print(
    framesShown
  );


  Serial.print(
    "  Lost: "
  );


  Serial.print(
    lostFrames
  );


  Serial.print(
    "  CRCBad: "
  );


  Serial.print(
    badCRC
  );


  Serial.print(
    "  QueueDrop: "
  );


  Serial.print(
    queueDrops
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


  Serial.print(
    outOfOrderFrames
  );


  Serial.print(
    "  Mode: "
  );


  Serial.print(
    realtimeActive ?
    "REALTIME" :
    "WLED"
  );


  Serial.print(
    "  WiFi: "
  );


  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    Serial.print(
      "CONNECTED"
    );


    uint8_t channel =
      0;


    wifi_second_chan_t secondary =
      WIFI_SECOND_CHAN_NONE;


    if (
      esp_wifi_get_channel(
        &channel,
        &secondary
      ) == ESP_OK
    )
    {
      Serial.print(
        " CH="
      );


      Serial.print(
        channel
      );
    }
  }
  else
  {
    Serial.print(
      "DISCONNECTED"
    );
  }


  Serial.println();
}


// ======================================================
// USERMOD
// ======================================================

class EspNowLedReceiverUsermod :
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
      "WLED ESP-NOW LED RECEIVER V3.1"
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


    Serial.println(
      "Output: WLED pixels 0-37"
    );


    Serial.println(
      "GPIO16: I2S"
    );


    Serial.println(
      "Incoming data: GRB"
    );


    Serial.println(
      "WLED Color Order: BRG"
    );


    Serial.println(
      "Logical mapping: RGB"
    );


    Serial.println(
      "ESP-NOW init: WAIT FOR WIFI"
    );


    Serial.println(
      "================================"
    );


    createRxQueue();


    Serial.println(
      "Waiting for WLED Wi-Fi..."
    );


    Serial.println(
      "================================"
    );
  }


  // ====================================================
  // LOOP
  // ====================================================

  void loop() override
  {
    uint32_t now =
      millis();


    // ==================================================
    // START ESP-NOW AFTER WLED WIFI
    // ==================================================

    if (
      !espNowReady &&
      WiFi.status() ==
      WL_CONNECTED
    )
    {
      if (
        now > 2000 &&
        now -
        lastInitAttemptMs >=
        3000
      )
      {
        lastInitAttemptMs =
          now;


        startEspNow();
      }
    }


    // ==================================================
    // RX + OUTPUT
    // ==================================================

    if (
      espNowReady
    )
    {
      processRxQueue();


      applyFrameToWLED();
    }


    // ==================================================
    // RESTORE WLED IF SIGNAL STOPS
    // ==================================================

    updateRealtimeTimeout();


    // ==================================================
    // STATUS
    // ==================================================

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
        "ESP-NOW LED RX"
      );


    info["ready"] =
      espNowReady;


    info["rx"] =
      packetsReceived;


    info["good"] =
      packetsGood;


    info["shown"] =
      framesShown;


    info["lost"] =
      lostFrames;


    info["crc_bad"] =
      badCRC;


    info["queue_drop"] =
      queueDrops;


    info["realtime"] =
      realtimeActive;
  }


  uint16_t getId() override
  {
    return 0x5057;
  }
};


// ======================================================
// REGISTER
// ======================================================

static EspNowLedReceiverUsermod
espNowLedReceiverUsermod;


REGISTER_USERMOD(
  espNowLedReceiverUsermod
);
