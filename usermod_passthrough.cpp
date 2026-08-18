#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_6

#define RMT_CLK_DIV 2

// ======================================================
// STRIP / FRAME
// ======================================================

#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)

// ======================================================
// MEASURED TIMING
// ======================================================

#define BIT_THRESHOLD 30

// المعدل الذي نجح سابقًا في الاستقبال المستمر
#define RMT_IDLE_TICKS 3000

// Print decoded frame once per second
#define FRAME_DEBUG_INTERVAL_MS 1000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;
  bool rxReady = false;

  uint8_t frameBuffer[FRAME_BYTES];

  uint32_t packetsReceived = 0;
  uint32_t goodFrames      = 0;
  uint32_t shortPackets    = 0;
  uint32_t longPackets     = 0;

  uint16_t lastPacketSymbols = 0;

  uint32_t lastDebugTime = 0;
  uint32_t lastFrameDebugTime = 0;


  // ====================================================
  // PRINT FRAME
  // ====================================================

  void printFrameDiagnostic()
  {
    uint32_t now = millis();

    if (
      now - lastFrameDebugTime <
      FRAME_DEBUG_INTERVAL_MS
    ) {
      return;
    }

    lastFrameDebugTime = now;

    Serial.println();
    Serial.println(
      "========== RX FRAME DATA =========="
    );

    for (
      uint8_t i = 0;
      i < WS2811_ICS;
      i++
    ) {
      uint8_t p = i * 3;

      Serial.print("IC");
      Serial.print(i + 1);

      Serial.print(": G=");
      Serial.print(frameBuffer[p + 0]);

      Serial.print(" R=");
      Serial.print(frameBuffer[p + 1]);

      Serial.print(" B=");
      Serial.println(frameBuffer[p + 2]);
    }

    Serial.println(
      "==================================="
    );
  }


  // ====================================================
  // DECODE ONE RMT PACKET
  // ====================================================

  bool decodePacket(
    rmt_item32_t *items,
    size_t count
  )
  {
    if (!items)
      return false;

    packetsReceived++;
    lastPacketSymbols = count;

    if (count < FRAME_BITS)
    {
      shortPackets++;
      return false;
    }

    if (count > FRAME_BITS + 8)
    {
      longPackets++;
    }

    memset(
      frameBuffer,
      0,
      sizeof(frameBuffer)
    );

    uint16_t bitIndex = 0;

    for (
      size_t i = 0;
      i < count && bitIndex < FRAME_BITS;
      i++
    )
    {
      if (
        items[i].duration0 == 0 &&
        items[i].duration1 == 0
      ) {
        continue;
      }

      bool bit =
        items[i].duration0 >= BIT_THRESHOLD;

      uint8_t byteIndex =
        bitIndex >> 3;

      uint8_t bitPosition =
        7 - (bitIndex & 0x07);

      if (bit)
      {
        frameBuffer[byteIndex] |=
          (1U << bitPosition);
      }

      bitIndex++;
    }

    if (bitIndex != FRAME_BITS)
    {
      shortPackets++;
      return false;
    }

    goodFrames++;

    return true;
  }


  // ====================================================
  // RMT SETUP
  // ====================================================

  bool setupRX()
  {
    rmt_config_t config = {};

    config.rmt_mode =
      RMT_MODE_RX;

    config.channel =
      RX_CHANNEL;

    config.gpio_num =
      (gpio_num_t)INPUT_GPIO;

    config.clk_div =
      RMT_CLK_DIV;

    config.mem_block_num =
      2;

    config.flags =
      0;

    config.rx_config.filter_en =
      false;

    // التعديل الرئيسي
    config.rx_config.idle_threshold =
      RMT_IDLE_TICKS;

    esp_err_t err;

    err =
      rmt_config(
        &config
      );

    if (err != ESP_OK)
    {
      Serial.print(
        "RMT CONFIG ERROR: "
      );

      Serial.println(err);

      return false;
    }

    err =
      rmt_driver_install(
        RX_CHANNEL,
        4096,
        0
      );

    if (err != ESP_OK)
    {
      Serial.print(
        "RMT DRIVER ERROR: "
      );

      Serial.println(err);

      return false;
    }

    err =
      rmt_get_ringbuf_handle(
        RX_CHANNEL,
        &rxRingBuffer
      );

    if (
      err != ESP_OK ||
      rxRingBuffer == nullptr
    )
    {
      Serial.println(
        "RMT RINGBUFFER ERROR"
      );

      return false;
    }

    err =
      rmt_rx_start(
        RX_CHANNEL,
        true
      );

    if (err != ESP_OK)
    {
      Serial.print(
        "RMT RX START ERROR: "
      );

      Serial.println(err);

      return false;
    }

    return true;
  }


public:

  void setup() override
  {
    pinMode(
      INPUT_GPIO,
      INPUT
    );

    memset(
      frameBuffer,
      0,
      sizeof(frameBuffer)
    );

    Serial.println();

    Serial.println(
      "================================"
    );

    Serial.println(
      "WS2811 RX-ONLY DIAGNOSTIC"
    );

    Serial.print(
      "Input GPIO: "
    );

    Serial.println(
      INPUT_GPIO
    );

    Serial.println(
      "RMT RX Channel: 6"
    );

    Serial.println(
      "RMT blocks: 2"
    );

    Serial.print(
      "Expected frame bits: "
    );

    Serial.println(
      FRAME_BITS
    );

    Serial.println(
      "Bit 0: H14~15"
    );

    Serial.println(
      "Bit 1: H50~51"
    );

    Serial.println(
      "Threshold: H >= 30"
    );

    Serial.print(
      "Idle threshold: "
    );

    Serial.println(
      RMT_IDLE_TICKS
    );

    Serial.println(
      "WLED output: DISABLED"
    );

    Serial.println(
      "Realtime: DISABLED"
    );

    Serial.println(
      "================================"
    );

    if (setupRX())
    {
      rxReady = true;

      Serial.println(
        "RMT RX READY"
      );
    }
    else
    {
      Serial.println(
        "RMT RX FAILED"
      );
    }
  }


  void loop() override
  {
    if (
      !rxReady ||
      rxRingBuffer == nullptr
    )
    {
      return;
    }

    size_t receivedSize = 0;

    rmt_item32_t *items =
      (rmt_item32_t *)
      xRingbufferReceive(
        rxRingBuffer,
        &receivedSize,
        0
      );

    if (items)
    {
      size_t count =
        receivedSize /
        sizeof(rmt_item32_t);

      if (
        decodePacket(
          items,
          count
        )
      )
      {
        printFrameDiagnostic();
      }

      vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
      );
    }

    uint32_t now = millis();

    if (
      now - lastDebugTime >= 2000
    )
    {
      lastDebugTime = now;

      Serial.print(
        "Packets: "
      );

      Serial.print(
        packetsReceived
      );

      Serial.print(
        "  Good: "
      );

      Serial.print(
        goodFrames
      );

      Serial.print(
        "  Short: "
      );

      Serial.print(
        shortPackets
      );

      Serial.print(
        "  Long: "
      );

      Serial.print(
        longPackets
      );

      Serial.print(
        "  Symbols: "
      );

      Serial.println(
        lastPacketSymbols
      );
    }
  }


  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "WS2811 RX Diagnostic"
      );

    info["input"] =
      INPUT_GPIO;

    info["rmt_channel"] =
      6;

    info["frame_bits"] =
      FRAME_BITS;

    info["idle_ticks"] =
      RMT_IDLE_TICKS;

    info["packets"] =
      packetsReceived;

    info["good"] =
      goodFrames;

    info["short"] =
      shortPackets;

    info["long"] =
      longPackets;

    info["last_symbols"] =
      lastPacketSymbols;
  }


  uint16_t getId() override
  {
    return 0x5043;
  }
};


static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
  passthroughUsermod
);
