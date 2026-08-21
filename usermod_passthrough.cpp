#include "wled.h"

#include <WiFi.h>
#include "esp_now.h"
#include "esp_wifi.h"

// ======================================================
// WLED DUAL ESP-NOW LED RX V2
// SYNCHRONIZED OUTPUT
//
// WLED 16.0.0
// ESP32 Classic / GL-C-016WL-D
//
// lineId 0:
//   pixels 0..37  -> GPIO16
//
// lineId 1:
//   pixels 38..75 -> GPIO2
//
// 38 IC per line
//
// Incoming packet order = GRB
// WLED physical Color Order = BRG
//
// Main V2 change:
// WAIT FOR BOTH LINES, THEN strip.show() ONCE.
// ======================================================


// ======================================================
// CONFIG
// ======================================================

#define ICS_PER_LINE 38
#define FRAME_BYTES (ICS_PER_LINE * 3)

#define PACKET_MAGIC   0x2811
#define PACKET_VERSION 2

#define RX_QUEUE_LENGTH 20

// Don't return to WLED unless transmitter really stops.
#define SIGNAL_TIMEOUT_MS 10000

#define REALTIME_LOCK_MS  2000

// Maximum wait for the other line.
// 50 ms ~= suitable for ~20 fps.
#define LINE_SYNC_TIMEOUT_MS 50


// ======================================================
// PACKET FORMAT
// ======================================================

struct __attribute__((packed))
LedPacket
{
  uint16_t magic;

  uint8_t version;

  uint8_t lineId;

  uint8_t icCount;

  uint32_t frameId;

  uint32_t txMicros;

  uint8_t data[FRAME_BYTES];

  uint16_t crc;
};


// ======================================================
// RX QUEUE
// ======================================================

struct RxQueueItem
{
  LedPacket packet;

  uint8_t senderMac[6];
};


static QueueHandle_t rxQueue = nullptr;


// ======================================================
// LINE BUFFERS
// ======================================================

static uint8_t lineFrame[2][FRAME_BYTES];

static bool lineReady[2] =
{
  false,
  false
};


// Last time each line received a valid packet.
static uint32_t lineReceivedMs[2] =
{
  0,
  0
};


// Used to start synchronization window.
static uint32_t syncWindowStartMs = 0;


// ======================================================
// ESP-NOW / REALTIME STATE
// ======================================================

static bool espNowReady = false;
static bool realtimeActive = false;

static uint32_t lastInitAttemptMs = 0;
static uint32_t lastPacketMillis = 0;


// ======================================================
// RX STATISTICS
// ======================================================

static volatile uint32_t packetsReceived = 0;
static volatile uint32_t badLength = 0;
static volatile uint32_t queueDrops = 0;

static uint32_t packetsGood[2] =
{
  0,
  0
};

static uint32_t framesShown[2] =
{
  0,
  0
};


// Number of actual strip.show() calls.
static uint32_t synchronizedShows = 0;


// Show performed because both lines were ready.
static uint32_t fullSyncShows = 0;


// Show performed because timeout expired.
static uint32_t timeoutShows = 0;


static uint32_t badMagic = 0;
static uint32_t badVersion = 0;
static uint32_t badIcCount = 0;
static uint32_t badLine = 0;
static uint32_t badCRC = 0;


// ======================================================
// SEQUENCE
// ======================================================

static uint32_t lastFrameId[2] =
{
  0,
  0
};

static uint32_t lostFrames[2] =
{
  0,
  0
};

static uint32_t duplicateFrames[2] =
{
  0,
  0
};

static uint32_t outOfOrderFrames[2] =
{
  0,
  0
};


// ======================================================
// DEBUG
// ======================================================

static uint32_t lastStatusMillis = 0;


// ======================================================
// CRC16 CCITT
// ======================================================

static uint16_t calculateCRC16(
  const uint8_t *data,
  size_t length
)
{
  uint16_t crc = 0xFFFF;


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
      if (crc & 0x8000)
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
// CREATE QUEUE
// ======================================================

static bool createRxQueue()
{
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
  if (espNowReady)
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


  uint8_t channel = 0;

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


  espNowReady = true;


  Serial.println(
    "ESP-NOW DUAL RX V2 READY"
  );


  return true;
}


// ======================================================
// SEQUENCE
// ======================================================

static void processSequence(
  uint8_t lineId,
  uint32_t frameId
)
{
  if (
    lastFrameId[lineId] == 0
  )
  {
    lastFrameId[lineId] =
      frameId;

    return;
  }


  if (
    frameId ==
    lastFrameId[lineId] + 1
  )
  {
    lastFrameId[lineId] =
      frameId;

    return;
  }


  if (
    frameId ==
    lastFrameId[lineId]
  )
  {
    duplicateFrames[lineId]++;

    return;
  }


  if (
    frameId >
    lastFrameId[lineId] + 1
  )
  {
    lostFrames[lineId] +=
      frameId -
      lastFrameId[lineId] -
      1;


    lastFrameId[lineId] =
      frameId;

    return;
  }


  outOfOrderFrames[lineId]++;
}


// ======================================================
// PROCESS PACKET
// ======================================================

static void processPacket(
  const RxQueueItem &item
)
{
  const LedPacket &packet =
    item.packet;


  if (
    packet.magic !=
    PACKET_MAGIC
  )
  {
    badMagic++;

    return;
  }


  if (
    packet.version !=
    PACKET_VERSION
  )
  {
    badVersion++;

    return;
  }


  if (
    packet.lineId > 1
  )
  {
    badLine++;

    return;
  }


  if (
    packet.icCount !=
    ICS_PER_LINE
  )
  {
    badIcCount++;

    return;
  }


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


  uint8_t lineId =
    packet.lineId;


  packetsGood[lineId]++;


  processSequence(
    lineId,
    packet.frameId
  );


  memcpy(
    lineFrame[lineId],
    packet.data,
    FRAME_BYTES
  );


  lineReady[lineId] = true;


  uint32_t now =
    millis();


  lineReceivedMs[lineId] =
    now;


  lastPacketMillis =
    now;


  // First line entering a new sync pair.
  if (
    !lineReady[
      lineId == 0 ? 1 : 0
    ]
  )
  {
    syncWindowStartMs =
      now;
  }
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
// COPY ONE LINE INTO WLED BUFFER
// ======================================================

static void copyLineToWLED(
  uint8_t lineId
)
{
  uint16_t pixelOffset =
    lineId *
    ICS_PER_LINE;


  for (
    uint16_t i = 0;
    i < ICS_PER_LINE;
    i++
  )
  {
    uint16_t p =
      i * 3;


    // Incoming packet = G R B

    uint8_t inputG =
      lineFrame[lineId][
        p + 0
      ];


    uint8_t inputR =
      lineFrame[lineId][
        p + 1
      ];


    uint8_t inputB =
      lineFrame[lineId][
        p + 2
      ];


    // Logical RGB.
    // WLED Color Order remains BRG.

    strip.setPixelColor(
      pixelOffset + i,
      inputR,
      inputG,
      inputB
    );
  }


  framesShown[lineId]++;
}


// ======================================================
// SYNCHRONIZED OUTPUT
// ======================================================

static void updateSynchronizedOutput()
{
  if (
    !lineReady[0] &&
    !lineReady[1]
  )
  {
    return;
  }


  uint32_t now =
    millis();


  bool bothReady =
    lineReady[0] &&
    lineReady[1];


  bool syncTimeout =
    (
      syncWindowStartMs != 0 &&
      now -
      syncWindowStartMs >=
      LINE_SYNC_TIMEOUT_MS
    );


  // Wait until:
  //
  // 1) BOTH lines are ready
  // OR
  // 2) 50 ms passed waiting for the other line.

  if (
    !bothReady &&
    !syncTimeout
  )
  {
    return;
  }


  realtimeLock(
    REALTIME_LOCK_MS,
    REALTIME_MODE_UDP
  );


  realtimeActive =
    true;


  // ----------------------------------------------------
  // IMPORTANT:
  //
  // Update BOTH WLED buffers first.
  // Then strip.show() only once.
  // ----------------------------------------------------

  if (
    lineReady[0]
  )
  {
    copyLineToWLED(
      0
    );
  }


  if (
    lineReady[1]
  )
  {
    copyLineToWLED(
      1
    );
  }


  strip.show();


  synchronizedShows++;


  if (bothReady)
  {
    fullSyncShows++;
  }
  else
  {
    timeoutShows++;
  }


  // Both pending states are now consumed.
  //
  // If one wasn't ready, its previous displayed frame
  // simply remains on the strip.

  lineReady[0] =
    false;

  lineReady[1] =
    false;


  syncWindowStartMs =
    0;
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


    lineReady[0] =
      false;

    lineReady[1] =
      false;


    syncWindowStartMs =
      0;


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
    "L0 RX="
  );

  Serial.print(
    packetsGood[0]
  );

  Serial.print(
    " Applied="
  );

  Serial.print(
    framesShown[0]
  );

  Serial.print(
    " Lost="
  );

  Serial.print(
    lostFrames[0]
  );


  Serial.print(
    " | L1 RX="
  );

  Serial.print(
    packetsGood[1]
  );

  Serial.print(
    " Applied="
  );

  Serial.print(
    framesShown[1]
  );

  Serial.print(
    " Lost="
  );

  Serial.println(
    lostFrames[1]
  );


  Serial.print(
    "Shows="
  );

  Serial.print(
    synchronizedShows
  );


  Serial.print(
    " FullSync="
  );

  Serial.print(
    fullSyncShows
  );


  Serial.print(
    " TimeoutShow="
  );

  Serial.println(
    timeoutShows
  );


  Serial.print(
    "TotalRX="
  );

  Serial.print(
    packetsReceived
  );


  Serial.print(
    " CRCBad="
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
    " BadLine="
  );

  Serial.print(
    badLine
  );


  Serial.print(
    " QueueDrop="
  );

  Serial.println(
    queueDrops
  );


  Serial.print(
    "Pending: L0="
  );

  Serial.print(
    lineReady[0] ?
    "YES" :
    "NO"
  );


  Serial.print(
    " L1="
  );

  Serial.println(
    lineReady[1] ?
    "YES" :
    "NO"
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

class DualEspNowSyncReceiver :
  public Usermod
{

public:

  void setup() override
  {
    memset(
      lineFrame,
      0,
      sizeof(lineFrame)
    );


    Serial.println();

    Serial.println(
      "======================================"
    );


    Serial.println(
      "WLED DUAL ESP-NOW LED RX V2"
    );


    Serial.println(
      "SYNCHRONIZED OUTPUT"
    );


    Serial.println(
      "======================================"
    );


    Serial.println(
      "lineId 0 -> GPIO16 / pixels 0..37"
    );


    Serial.println(
      "lineId 1 -> GPIO2 / pixels 38..75"
    );


    Serial.println(
      "38 IC per line"
    );


    Serial.println(
      "Incoming = GRB"
    );


    Serial.println(
      "WLED Color Order = BRG"
    );


    Serial.print(
      "Sync wait = "
    );

    Serial.print(
      LINE_SYNC_TIMEOUT_MS
    );

    Serial.println(
      " ms"
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
      sizeof(LedPacket)
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
    // ESP-NOW after WLED Wi-Fi
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
    // RECEIVE + SYNCHRONIZED OUTPUT
    // ==================================================

    if (
      espNowReady
    )
    {
      processRxQueue();


      updateSynchronizedOutput();
    }


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


  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "Dual Sync ESP-NOW RX"
      );


    info["ready"] =
      espNowReady;


    info["l0_rx"] =
      packetsGood[0];


    info["l1_rx"] =
      packetsGood[1];


    info["shows"] =
      synchronizedShows;


    info["full_sync"] =
      fullSyncShows;


    info["timeout_show"] =
      timeoutShows;


    info["crc_bad"] =
      badCRC;


    info["queue_drop"] =
      queueDrops;


    info["realtime"] =
      realtimeActive;
  }


  uint16_t getId() override
  {
    return 0x5061;
  }
};


// ======================================================
// REGISTER
// ======================================================

static DualEspNowSyncReceiver
dualEspNowSyncReceiver;


REGISTER_USERMOD(
  dualEspNowSyncReceiver
);
