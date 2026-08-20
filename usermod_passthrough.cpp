#include "wled.h"

#include <WiFi.h>
#include "esp_now.h"
#include "esp_wifi.h"

// ======================================================
// WLED DUAL ESP-NOW LED RECEIVER V1
//
// WLED 16.0.0
// ESP32 Classic / GL-C-016WL-D
//
// lineId 0:
//   38 IC -> WLED pixels 0..37 -> GPIO16
//
// lineId 1:
//   38 IC -> WLED pixels 38..75 -> GPIO2
//
// Incoming WS2811 byte order:
//   G R B
//
// WLED logical API:
//   R G B
//
// WLED physical Color Order:
//   BRG
// ======================================================


// ======================================================
// CONFIG
// ======================================================

#define ICS_PER_LINE 38

#define FRAME_BYTES \
  (ICS_PER_LINE * 3)

#define TOTAL_WLED_PIXELS \
  (ICS_PER_LINE * 2)

#define PACKET_MAGIC   0x2811
#define PACKET_VERSION 2

#define RX_QUEUE_LENGTH 16

#define SIGNAL_TIMEOUT_MS 1500
#define REALTIME_LOCK_MS  1000


// ======================================================
// PACKET
//
// MUST MATCH DUAL TX V1
//
// total = 129 bytes
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

  uint8_t data[
    FRAME_BYTES
  ];

  uint16_t crc;
};


// ======================================================
// QUEUE ITEM
// ======================================================

struct RxQueueItem
{
  LedPacket packet;

  uint8_t senderMac[6];
};


static QueueHandle_t rxQueue =
  nullptr;


// ======================================================
// FRAME BUFFERS
// ======================================================

static uint8_t lineFrame[2][FRAME_BYTES];

static bool lineFrameReady[2] =
{
  false,
  false
};


// ======================================================
// ESP-NOW STATE
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
// RX STATS
// ======================================================

static volatile uint32_t packetsReceived =
  0;

static volatile uint32_t badLength =
  0;

static volatile uint32_t queueDrops =
  0;


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


static uint32_t badMagic =
  0;

static uint32_t badVersion =
  0;

static uint32_t badIcCount =
  0;

static uint32_t badLine =
  0;

static uint32_t badCRC =
  0;


// ======================================================
// SEQUENCE STATS - PER LINE
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

static uint32_t lastStatusMillis =
  0;


// ======================================================
// CRC16 CCITT
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
    "ESP-NOW DUAL RECEIVER READY"
  );


  return true;
}


// ======================================================
// FRAME SEQUENCE
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


  // ----------------------------------------------------
  // MAGIC
  // ----------------------------------------------------

  if (
    packet.magic !=
    PACKET_MAGIC
  )
  {
    badMagic++;

    return;
  }


  // ----------------------------------------------------
  // VERSION
  // ----------------------------------------------------

  if (
    packet.version !=
    PACKET_VERSION
  )
  {
    badVersion++;

    return;
  }


  // ----------------------------------------------------
  // LINE
  // ----------------------------------------------------

  if (
    packet.lineId > 1
  )
  {
    badLine++;

    return;
  }


  // ----------------------------------------------------
  // IC COUNT
  // ----------------------------------------------------

  if (
    packet.icCount !=
    ICS_PER_LINE
  )
  {
    badIcCount++;

    return;
  }


  // ----------------------------------------------------
  // CRC
  // ----------------------------------------------------

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
  // GOOD FRAME
  // ====================================================

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


  lineFrameReady[lineId] =
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
    processPacket(
      item
    );
  }
}


// ======================================================
// APPLY ONE LINE TO WLED
// ======================================================

static void applyLineToWLED(
  uint8_t lineId
)
{
  if (
    !lineFrameReady[lineId]
  )
  {
    return;
  }


  // ----------------------------------------------------
  // line 0 starts at WLED pixel 0
  // line 1 starts at WLED pixel 38
  // ----------------------------------------------------

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


    // Incoming:
    // byte 0 = G
    // byte 1 = R
    // byte 2 = B

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


    // Logical RGB to WLED.
    //
    // WLED itself handles physical BRG ordering.

    strip.setPixelColor(
      pixelOffset + i,
      inputR,
      inputG,
      inputB
    );
  }


  lineFrameReady[lineId] =
    false;


  framesShown[lineId]++;
}


// ======================================================
// APPLY READY FRAMES
// ======================================================

static void applyFramesToWLED()
{
  if (
    !lineFrameReady[0] &&
    !lineFrameReady[1]
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


  // Update whichever line has a new frame.
  applyLineToWLED(
    0
  );


  applyLineToWLED(
    1
  );


  // Send both configured WLED buses.
  strip.show();
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


    lineFrameReady[0] =
      false;

    lineFrameReady[1] =
      false;


    exitRealtime();


    Serial.println(
      "ESP-NOW signal lost -> WLED resumed"
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
    " Shown="
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
    " Shown="
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
    "Total RX="
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

class DualEspNowLedReceiver :
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
      "WLED DUAL ESP-NOW LED RX V1"
    );


    Serial.println(
      "WLED 16.0.0 / ESP32 Classic"
    );


    Serial.println(
      "======================================"
    );


    Serial.println(
      "lineId 0 -> pixels 0..37 -> GPIO16"
    );


    Serial.println(
      "lineId 1 -> pixels 38..75 -> GPIO2"
    );


    Serial.println(
      "38 IC per line"
    );


    Serial.println(
      "Incoming order = GRB"
    );


    Serial.println(
      "WLED Color Order = BRG"
    );


    Serial.println(
      "Packet version = 2"
    );


    Serial.print(
      "Expected packet bytes = "
    );


    Serial.println(
      sizeof(LedPacket)
    );


    Serial.println(
      "ESP-NOW: WAIT FOR WIFI"
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


    // --------------------------------------------------
    // Start ESP-NOW only after WLED Wi-Fi.
    // --------------------------------------------------

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


    // --------------------------------------------------
    // RX
    // --------------------------------------------------

    if (
      espNowReady
    )
    {
      processRxQueue();

      applyFramesToWLED();
    }


    updateRealtimeTimeout();


    // --------------------------------------------------
    // STATUS
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


  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "Dual ESP-NOW RX"
      );


    info["ready"] =
      espNowReady;


    info["line0_rx"] =
      packetsGood[0];


    info["line1_rx"] =
      packetsGood[1];


    info["line0_shown"] =
      framesShown[0];


    info["line1_shown"] =
      framesShown[1];


    info["line0_lost"] =
      lostFrames[0];


    info["line1_lost"] =
      lostFrames[1];


    info["crc_bad"] =
      badCRC;


    info["queue_drop"] =
      queueDrops;


    info["realtime"] =
      realtimeActive;
  }


  uint16_t getId() override
  {
    return 0x5060;
  }
};


// ======================================================
// REGISTER
// ======================================================

static DualEspNowLedReceiver
dualEspNowLedReceiver;


REGISTER_USERMOD(
  dualEspNowLedReceiver
);
