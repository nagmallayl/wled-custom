#include "wled.h"

#include <WiFi.h>
#include "esp_now.h"
#include "esp_wifi.h"

// ======================================================
// WLED DUAL ESP-NOW LED RX V5.0
// COMBINED PAIR PACKET
//
// WLED 16.0.0
// ESP32 Classic / GL-C-016WL-D
//
// TX Packet Version = 3
//
// line0:
//   pixels 0..37
//
// line1:
//   pixels 38..75
//
// 38 IC per line
// 76 IC total
//
// Incoming packet order = GRB
// WLED physical Color Order = BRG
//
// V5.0:
// - Receives BOTH lines in ONE ESP-NOW packet
// - No RX timestamp matching
// - No RX line pairing
// - No partial frames
// - One CRC protects both lines
// - Both lines copied before output
// - Exactly ONE strip.show() per valid packet
// ======================================================


// ======================================================
// CONFIG
// ======================================================

#define ICS_PER_LINE 38

#define TOTAL_ICS (ICS_PER_LINE * 2)

#define FRAME_BYTES (ICS_PER_LINE * 3)

#define PACKET_MAGIC   0x2811

#define PACKET_VERSION 3

#define RX_QUEUE_LENGTH 12

#define SIGNAL_TIMEOUT_MS 10000

#define REALTIME_LOCK_MS 2000


// ======================================================
// PACKET FORMAT
//
// MUST MATCH TX V2.0 EXACTLY
// ======================================================

struct __attribute__((packed)) CombinedLedPacket
{
  uint16_t magic;

  uint8_t version;

  uint8_t icCount;

  uint32_t pairFrameId;

  uint32_t txMicros;

  uint8_t line0[FRAME_BYTES];

  uint8_t line1[FRAME_BYTES];

  uint16_t crc;
};


// ======================================================
// RX QUEUE ITEM
// ======================================================

struct RxQueueItem
{
  CombinedLedPacket packet;

  uint8_t senderMac[6];
};


static QueueHandle_t rxQueue =
  nullptr;


// ======================================================
// ESP-NOW / REALTIME
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


static uint32_t packetsGood =
  0;

static uint32_t framesApplied =
  0;


static uint32_t badMagic =
  0;

static uint32_t badVersion =
  0;

static uint32_t badIcCount =
  0;

static uint32_t badCRC =
  0;


// ======================================================
// SEQUENCE
// ======================================================

static uint32_t lastPairFrameId =
  0;


static uint32_t lostPairs =
  0;


static uint32_t duplicatePairs =
  0;


static uint32_t outOfOrderPairs =
  0;


// ======================================================
// TIMING / PERFORMANCE
// ======================================================

static uint32_t lastTxMicros =
  0;


static uint32_t lastPacketIntervalUs =
  0;


static uint32_t maxPacketIntervalUs =
  0;


static uint32_t lastShowMicros =
  0;


static uint32_t lastShowIntervalUs =
  0;


static uint32_t maxShowIntervalUs =
  0;


// ======================================================
// DEBUG
// ======================================================

static uint32_t lastStatusMillis =
  0;


// ======================================================
// CRC16 CCITT
//
// MUST MATCH TX
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
        crc <<= 1;
      }
    }
  }


  return crc;
}


// ======================================================
// ESP-NOW CALLBACK
//
// Keep callback short.
// Only copy packet into queue.
// ======================================================

static void onEspNowReceive(
  const uint8_t *macAddress,
  const uint8_t *incomingData,
  int len
)
{
  packetsReceived++;


  if (
    len != sizeof(CombinedLedPacket)
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
    sizeof(CombinedLedPacket)
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
    "ESP-NOW DUAL RX V5.0 READY"
  );


  return true;
}


// ======================================================
// PROCESS PAIR SEQUENCE
// ======================================================

static void processPairSequence(
  uint32_t frameId
)
{
  if (
    lastPairFrameId == 0
  )
  {
    lastPairFrameId =
      frameId;

    return;
  }


  if (
    frameId ==
    lastPairFrameId + 1
  )
  {
    lastPairFrameId =
      frameId;

    return;
  }


  if (
    frameId ==
    lastPairFrameId
  )
  {
    duplicatePairs++;

    return;
  }


  if (
    frameId >
    lastPairFrameId + 1
  )
  {
    lostPairs +=
      frameId -
      lastPairFrameId -
      1;


    lastPairFrameId =
      frameId;


    return;
  }


  outOfOrderPairs++;
}


// ======================================================
// COPY ONE LINE TO WLED
// ======================================================

static void copyLineToWLED(
  const uint8_t *frame,
  uint16_t pixelOffset
)
{
  for (
    uint16_t i = 0;
    i < ICS_PER_LINE;
    i++
  )
  {
    uint16_t p =
      i * 3;


    // Incoming WS2811 data = GRB

    uint8_t inputG =
      frame[p + 0];


    uint8_t inputR =
      frame[p + 1];


    uint8_t inputB =
      frame[p + 2];


    // WLED API receives logical RGB.
    //
    // Physical LED Color Order remains BRG
    // in WLED LED Preferences.

    strip.setPixelColor(
      pixelOffset + i,
      inputR,
      inputG,
      inputB
    );
  }
}


// ======================================================
// APPLY COMPLETE COMBINED PACKET
//
// IMPORTANT:
// Both lines are already inside the SAME packet.
// No matching or waiting is required here.
// ======================================================

static void applyCombinedPacket(
  const CombinedLedPacket &packet
)
{
  realtimeLock(
    REALTIME_LOCK_MS,
    REALTIME_MODE_UDP
  );


  realtimeActive =
    true;


  // ----------------------------------------------------
  // First half
  // line0 -> pixels 0..37
  // ----------------------------------------------------

  copyLineToWLED(
    packet.line0,
    0
  );


  // ----------------------------------------------------
  // Second half
  // line1 -> pixels 38..75
  // ----------------------------------------------------

  copyLineToWLED(
    packet.line1,
    ICS_PER_LINE
  );


  // ----------------------------------------------------
  // IMPORTANT
  //
  // BOTH halves have now been updated in WLED's buffer.
  // Send them physically together.
  // ----------------------------------------------------

  strip.show();


  framesApplied++;


  uint32_t nowUs =
    micros();


  if (
    lastShowMicros != 0
  )
  {
    lastShowIntervalUs =
      nowUs -
      lastShowMicros;


    if (
      lastShowIntervalUs >
      maxShowIntervalUs
    )
    {
      maxShowIntervalUs =
        lastShowIntervalUs;
    }
  }


  lastShowMicros =
    nowUs;
}


// ======================================================
// PROCESS PACKET
// ======================================================

static void processPacket(
  const RxQueueItem &item
)
{
  const CombinedLedPacket &packet =
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
    ICS_PER_LINE
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
      sizeof(packet) -
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
  // GOOD COMPLETE PAIR PACKET
  // ====================================================

  packetsGood++;


  processPairSequence(
    packet.pairFrameId
  );


  // ====================================================
  // TX TIMING DIAGNOSTIC
  // ====================================================

  if (
    lastTxMicros != 0
  )
  {
    lastPacketIntervalUs =
      packet.txMicros -
      lastTxMicros;


    if (
      lastPacketIntervalUs >
      maxPacketIntervalUs
    )
    {
      maxPacketIntervalUs =
        lastPacketIntervalUs;
    }
  }


  lastTxMicros =
    packet.txMicros;


  // ====================================================
  // APPLY BOTH LINES
  // ====================================================

  applyCombinedPacket(
    packet
  );


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
    processPacket(
      item
    );
  }
}


// ======================================================
// SIGNAL TIMEOUT
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


    exitRealtime();


    Serial.println(
      "ESP-NOW stopped >10 sec -> WLED resumed"
    );
  }
}


// ======================================================
// STATUS
// ======================================================

static void printStatus()
{
  Serial.println();


  Serial.print(
    "RX="
  );

  Serial.print(
    packetsReceived
  );


  Serial.print(
    " Good="
  );

  Serial.print(
    packetsGood
  );


  Serial.print(
    " Applied="
  );

  Serial.println(
    framesApplied
  );


  Serial.print(
    "PairFrame="
  );

  Serial.print(
    lastPairFrameId
  );


  Serial.print(
    " Lost="
  );

  Serial.print(
    lostPairs
  );


  Serial.print(
    " Duplicate="
  );

  Serial.print(
    duplicatePairs
  );


  Serial.print(
    " OutOfOrder="
  );

  Serial.println(
    outOfOrderPairs
  );


  Serial.print(
    "BadCRC="
  );

  Serial.print(
    badCRC
  );


  Serial.print(
    " BadLen="
  );

  Serial.print(
    badLength
  );


  Serial.print(
    " BadMagic="
  );

  Serial.print(
    badMagic
  );


  Serial.print(
    " BadVersion="
  );

  Serial.print(
    badVersion
  );


  Serial.print(
    " BadIC="
  );

  Serial.print(
    badIcCount
  );


  Serial.print(
    " QueueDrop="
  );

  Serial.println(
    queueDrops
  );


  Serial.print(
    "TX interval us: Last="
  );

  Serial.print(
    lastPacketIntervalUs
  );


  Serial.print(
    " Max="
  );

  Serial.println(
    maxPacketIntervalUs
  );


  Serial.print(
    "SHOW interval us: Last="
  );

  Serial.print(
    lastShowIntervalUs
  );


  Serial.print(
    " Max="
  );

  Serial.println(
    maxShowIntervalUs
  );


  Serial.print(
    "Packet bytes="
  );

  Serial.println(
    sizeof(CombinedLedPacket)
  );


  Serial.print(
    "Mode="
  );


  Serial.print(
    realtimeActive ?
    "REALTIME" :
    "WLED"
  );


  Serial.print(
    " WiFi="
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

class DualEspNowCombinedReceiver :
  public Usermod
{

public:

  void setup() override
  {
    Serial.println();


    Serial.println(
      "======================================"
    );


    Serial.println(
      "WLED DUAL ESP-NOW LED RX V5.0"
    );


    Serial.println(
      "COMBINED PAIR PACKET"
    );


    Serial.println(
      "======================================"
    );


    Serial.println(
      "Packet Version = 3"
    );


    Serial.println(
      "line0 -> pixels 0..37"
    );


    Serial.println(
      "line1 -> pixels 38..75"
    );


    Serial.println(
      "38 IC per line / 76 total"
    );


    Serial.println(
      "Incoming = GRB"
    );


    Serial.println(
      "WLED Color Order = BRG"
    );


    Serial.println(
      "RX Timestamp Matching = DISABLED"
    );


    Serial.println(
      "RX Line Pairing = DISABLED"
    );


    Serial.println(
      "Partial Show = DISABLED"
    );


    Serial.println(
      "Combined Packet = ENABLED"
    );


    Serial.println(
      "One strip.show() per packet"
    );


    Serial.print(
      "Signal timeout = "
    );


    Serial.print(
      SIGNAL_TIMEOUT_MS
    );


    Serial.println(
      " ms"
    );


    Serial.print(
      "Packet bytes = "
    );


    Serial.println(
      sizeof(CombinedLedPacket)
    );


    Serial.println(
      "======================================"
    );


    createRxQueue();


    Serial.println(
      "Waiting for WLED Wi-Fi..."
    );
  }


  void loop() override
  {
    uint32_t now =
      millis();


    // ==================================================
    // Start ESP-NOW after WLED connects to Wi-Fi
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
    // Receive complete pair packets
    // ==================================================

    if (
      espNowReady
    )
    {
      processRxQueue();
    }


    // ==================================================
    // Realtime timeout
    // ==================================================

    updateRealtimeTimeout();


    // ==================================================
    // Status every 2 seconds
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
  // WLED INFO PAGE
  // ====================================================

  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "Dual Combined ESP-NOW RX"
      );


    info["ready"] =
      espNowReady;


    info["rx"] =
      packetsReceived;


    info["good"] =
      packetsGood;


    info["applied"] =
      framesApplied;


    info["pair_frame"] =
      lastPairFrameId;


    info["lost"] =
      lostPairs;


    info["duplicate"] =
      duplicatePairs;


    info["out_of_order"] =
      outOfOrderPairs;


    info["crc_bad"] =
      badCRC;


    info["bad_len"] =
      badLength;


    info["queue_drop"] =
      queueDrops;


    info["packet_bytes"] =
      sizeof(CombinedLedPacket);


    info["realtime"] =
      realtimeActive;
  }


  uint16_t getId() override
  {
    return 0x5067;
  }
};


// ======================================================
// REGISTER USERMOD
// ======================================================

static DualEspNowCombinedReceiver
  dualEspNowCombinedReceiver;


REGISTER_USERMOD(
  dualEspNowCombinedReceiver
);
