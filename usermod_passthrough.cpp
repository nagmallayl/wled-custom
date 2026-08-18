#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32 classic
// 80 MHz / 2 = 40 MHz
#define RMT_CLK_DIV 2

// ======================================================
// STRIP
// ======================================================

// 15 physical LEDs
// 5 WS2811 ICs
#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)

// ======================================================
// REAL MEASURED TIMING
// ======================================================
//
// From your diagnostic:
//
// 0:
// H = 14~15
//
// 1:
// H = 50~51
//
// So:
//
// H < 30  = 0
// H >= 30 = 1
// ======================================================

#define BIT_THRESHOLD 30

// Auto return to WLED if source disappears
#define SIGNAL_TIMEOUT_MS 2000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ====================================================
  // RX DECODER
  // ====================================================

  uint8_t rxFrame[FRAME_BYTES];

  uint8_t currentByte = 0;
  uint8_t bitCount = 0;
  uint8_t frameBytes = 0;

  // ====================================================
  // LAST GOOD FRAME
  // ====================================================

  uint8_t latestFrame[FRAME_BYTES];

  bool haveFrame = false;
  bool passthroughActive = false;

  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;
  uint32_t framesApplied = 0;

  uint32_t lastFrameTime = 0;
  uint32_t lastDebugTime = 0;

  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    currentByte = 0;
    bitCount = 0;
    frameBytes = 0;
  }

  // ====================================================
  // COMMIT FRAME
  // ====================================================

  void commitFrame()
  {
    if (
        frameBytes != FRAME_BYTES ||
        bitCount != 0
    ) {
      return;
    }

    memcpy(
        latestFrame,
        rxFrame,
        FRAME_BYTES
    );

    haveFrame = true;
    passthroughActive = true;

    framesReceived++;

    lastFrameTime = millis();

    resetDecoder();
  }

  // ====================================================
  // PROCESS RMT
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count
  )
  {
    if (!items)
      return;

    for (
        size_t i = 0;
        i < count;
        i++
    )
    {
      uint16_t high =
          items[i].duration0;

      // ----------------------------------------------
      // Ignore completely empty symbol
      // ----------------------------------------------

      if (
          items[i].duration0 == 0 &&
          items[i].duration1 == 0
      ) {
        continue;
      }

      /*
       * IMPORTANT:
       *
       * We deliberately DO NOT validate:
       *
       * high + low
       *
       * because your actual measured signal showed:
       *
       * bit 0 total ≈ 56
       * bit 1 total ≈ 75
       *
       * We also do NOT use level0/level1 here.
       *
       * This matches the decoder version that gave
       * you stable Frames with Dropped = 0.
       */

      bool bit =
          high >= BIT_THRESHOLD;

      currentByte <<= 1;

      if (bit) {
        currentByte |= 1;
      }

      bitCount++;

      // ----------------------------------------------
      // Complete byte
      // ----------------------------------------------

      if (bitCount == 8)
      {
        if (
            frameBytes < FRAME_BYTES
        )
        {
          rxFrame[frameBytes] =
              currentByte;

          frameBytes++;
        }

        currentByte = 0;
        bitCount = 0;
      }

      // ----------------------------------------------
      // Complete fixed-size frame
      //
      // 15 bytes = 120 bits
      // ----------------------------------------------

      if (
          frameBytes == FRAME_BYTES &&
          bitCount == 0
      )
      {
        commitFrame();
      }
    }
  }

  // ====================================================
  // APPLY LAST FRAME TO WLED
  // ====================================================

  void applyLatestFrame()
  {
    if (
        !passthroughActive ||
        !haveFrame
    ) {
      return;
    }

    for (
        uint8_t i = 0;
        i < WS2811_ICS;
        i++
    )
    {
      uint8_t p = i * 3;

      // Incoming:
      // G R B

      uint8_t inputG =
          latestFrame[p + 0];

      uint8_t inputR =
          latestFrame[p + 1];

      uint8_t inputB =
          latestFrame[p + 2];

      // =================================================
      // GRB -> BRG
      // =================================================
      //
      // Put BRG into WLED RGB buffer:
      //
      // R = B
      // G = R
      // B = G
      // =================================================

      uint8_t outputR =
          inputB;

      uint8_t outputG =
          inputR;

      uint8_t outputB =
          inputG;

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }

    /*
     * IMPORTANT:
     *
     * NO strip.show()
     * NO strip.trigger()
     *
     * WLED itself sends the frame.
     */

    framesApplied++;
  }

  // ====================================================
  // SETUP RMT RX
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

    /*
     * Same configuration that previously
     * produced stable Frames.
     */

    config.mem_block_num = 2;

    config.flags = 0;

    config.rx_config.filter_en =
        false;

    config.rx_config.idle_threshold =
        3000;

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

  // ====================================================
  // SETUP
  // ====================================================

  void setup() override
  {
    pinMode(
        INPUT_GPIO,
        INPUT
    );

    memset(
        rxFrame,
        0,
        sizeof(rxFrame)
    );

    memset(
        latestFrame,
        0,
        sizeof(latestFrame)
    );

    Serial.println();

    Serial.println(
        "================================"
    );

    Serial.println(
        "Passthrough Usermod STARTED"
    );

    Serial.print(
        "Input GPIO: "
    );

    Serial.println(
        INPUT_GPIO
    );

    Serial.println(
        "Output: WLED"
    );

    Serial.println(
        "RMT: LEGACY RX"
    );

    Serial.println(
        "RMT RX Channel: 6"
    );

    Serial.println(
        "RMT blocks: 2"
    );

    Serial.println(
        "Decoder: SIMPLE duration0"
    );

    Serial.print(
        "Physical LEDs: "
    );

    Serial.println(
        PHYSICAL_LEDS
    );

    Serial.print(
        "WS2811 ICs: "
    );

    Serial.println(
        WS2811_ICS
    );

    Serial.print(
        "Frame bytes: "
    );

    Serial.println(
        FRAME_BYTES
    );

    Serial.print(
        "Frame bits: "
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

    Serial.println(
        "Input: GRB"
    );

    Serial.println(
        "Output: BRG"
    );

    Serial.println(
        "strip.show(): DISABLED"
    );

    Serial.println(
        "strip.trigger(): DISABLED"
    );

    Serial.println(
        "Output: WLED OVERLAY"
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

      return;
    }

    Serial.println(
        "Passthrough READY"
    );

    Serial.println(
        "Waiting for WS2811 DATA..."
    );

    Serial.println(
        "================================"
    );
  }

  // ====================================================
  // WLED OVERLAY
  // ====================================================

  void handleOverlayDraw() override
  {
    if (
        passthroughActive &&
        haveFrame
    )
    {
      /*
       * Always overwrite WLED effect colors
       * with the latest good GPIO25 frame.
       */

      applyLatestFrame();
    }
  }

  // ====================================================
  // LOOP
  // ====================================================

  void loop() override
  {
    if (
        !rxReady ||
        rxRingBuffer == nullptr
    ) {
      return;
    }

    size_t receivedSize = 0;

    // Non-blocking
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

      processSymbols(
          items,
          count
      );

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }

    // =================================================
    // AUTO RETURN TO WLED
    // =================================================

    if (passthroughActive)
    {
      uint32_t now =
          millis();

      if (
          now - lastFrameTime >
          SIGNAL_TIMEOUT_MS
      )
      {
        passthroughActive = false;

        haveFrame = false;

        resetDecoder();

        Serial.println(
            "Passthrough OFF -> WLED"
        );
      }
    }

    // =================================================
    // DEBUG
    // =================================================

    uint32_t now =
        millis();

    if (
        now - lastDebugTime >= 2000
    )
    {
      lastDebugTime = now;

      Serial.print(
          "Frames: "
      );

      Serial.print(
          framesReceived
      );

      Serial.print(
          "  Applied: "
      );

      Serial.print(
          framesApplied
      );

      Serial.print(
          "  RX Bytes: "
      );

      Serial.print(
          frameBytes
      );

      Serial.print(
          "  Bits: "
      );

      Serial.print(
          bitCount
      );

      Serial.print(
          "  Mode: "
      );

      if (passthroughActive)
      {
        Serial.println(
            "PASSTHROUGH"
        );
      }
      else
      {
        Serial.println(
            "WLED"
        );
      }
    }
  }

  // ====================================================
  // INFO
  // ====================================================

  void addToJsonInfo(
      JsonObject &root
  ) override
  {
    JsonObject info =
        root["u"]
        .createNestedObject(
            "Passthrough"
        );

    info["input"] =
        INPUT_GPIO;

    info["output"] =
        "WLED";

    info["rmt_channel"] =
        6;

    info["physical_leds"] =
        PHYSICAL_LEDS;

    info["ws2811_ics"] =
        WS2811_ICS;

    info["frame_bits"] =
        FRAME_BITS;

    info["input_order"] =
        "GRB";

    info["output_order"] =
        "BRG";

    info["frames"] =
        framesReceived;

    info["applied"] =
        framesApplied;
  }

  uint16_t getId() override
  {
    return 0x5041;
  }
};


static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
    passthroughUsermod
);
