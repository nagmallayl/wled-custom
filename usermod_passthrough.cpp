#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_6

#define RMT_CLK_DIV 2

// ======================================================
// STRIP
// ======================================================

#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)

// ======================================================
// MEASURED TIMING
// ======================================================
//
// 0:
// H ≈ 14~15
//
// 1:
// H ≈ 50~51
//
#define BIT_THRESHOLD 30

// ======================================================
// REALTIME
// ======================================================

#define REALTIME_LOCK_MS  1000
#define SIGNAL_TIMEOUT_MS 1500

// ======================================================
// DIAGNOSTIC
// ======================================================

// Print one decoded frame approximately once per second.
#define FRAME_DEBUG_INTERVAL_MS 1000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;
  bool rxReady = false;

  // ====================================================
  // FRAME ASSEMBLER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  uint8_t currentByte = 0;
  uint8_t bitCount    = 0;
  uint8_t frameBytes  = 0;

  uint16_t frameSymbols = 0;

  // ====================================================
  // STATE
  // ====================================================

  bool realtimeActive = false;

  uint32_t lastFrameTime = 0;

  // ====================================================
  // STATS
  // ====================================================

  uint32_t chunksReceived = 0;
  uint32_t framesReceived = 0;
  uint32_t framesShown    = 0;
  uint32_t resyncCount    = 0;

  uint32_t lastDebugTime = 0;

  // ====================================================
  // FRAME DATA DEBUG
  // ====================================================

  uint32_t lastFrameDebugTime = 0;

  // ====================================================
  // RESET ASSEMBLER
  // ====================================================

  void resetAssembler()
  {
    currentByte  = 0;
    bitCount     = 0;
    frameBytes   = 0;
    frameSymbols = 0;
  }

  // ====================================================
  // REALTIME
  // ====================================================

  void keepRealtimeActive()
  {
    realtimeLock(
      REALTIME_LOCK_MS,
      REALTIME_MODE_UDP
    );

    realtimeActive = true;
  }

  // ====================================================
  // PRINT DECODED FRAME
  // ====================================================

  void printFrameDiagnostic()
  {
    uint32_t now = millis();

    if (
      now - lastFrameDebugTime <
      FRAME_DEBUG_INTERVAL_MS
    )
    {
      return;
    }

    lastFrameDebugTime = now;

    Serial.println();
    Serial.println(
      "========== FRAME DATA =========="
    );

    for (
      uint8_t i = 0;
      i < WS2811_ICS;
      i++
    )
    {
      uint8_t p = i * 3;

      uint8_t g =
        frameBuffer[p + 0];

      uint8_t r =
        frameBuffer[p + 1];

      uint8_t b =
        frameBuffer[p + 2];

      Serial.print("IC");
      Serial.print(i + 1);

      Serial.print(": G=");
      Serial.print(g);

      Serial.print(" R=");
      Serial.print(r);

      Serial.print(" B=");
      Serial.println(b);
    }

    Serial.println(
      "================================"
    );
  }

  // ====================================================
  // SHOW COMPLETE FRAME
  // ====================================================

  void showCompleteFrame()
  {
    if (
      frameBytes != FRAME_BYTES ||
      bitCount != 0
    )
    {
      resetAssembler();
      return;
    }

    // --------------------------------------------------
    // Diagnostic BEFORE changing/resetting the buffer
    // --------------------------------------------------

    printFrameDiagnostic();

    keepRealtimeActive();

    // ==================================================
    // INPUT = GRB
    // OUTPUT = BRG
    // ==================================================

    for (
      uint8_t i = 0;
      i < WS2811_ICS;
      i++
    )
    {
      uint8_t p = i * 3;

      uint8_t inputG =
        frameBuffer[p + 0];

      uint8_t inputR =
        frameBuffer[p + 1];

      uint8_t inputB =
        frameBuffer[p + 2];

      // GRB -> BRG
      uint8_t outputR = inputB;
      uint8_t outputG = inputR;
      uint8_t outputB = inputG;

      strip.setPixelColor(
        i,
        outputR,
        outputG,
        outputB
      );
    }

    strip.show();

    framesShown++;
    framesReceived++;

    lastFrameTime = millis();

    resetAssembler();
  }

  // ====================================================
  // PROCESS ONE RMT SYMBOL
  // ====================================================

  void processSymbol(
    const rmt_item32_t &item
  )
  {
    if (
      item.duration0 == 0 &&
      item.duration1 == 0
    )
    {
      return;
    }

    /*
     * Keep the decoder that gave the best
     * results in previous tests:
     *
     * duration0 < 30  = 0
     * duration0 >= 30 = 1
     */

    bool bit =
      item.duration0 >= BIT_THRESHOLD;

    currentByte <<= 1;

    if (bit)
    {
      currentByte |= 1;
    }

    bitCount++;
    frameSymbols++;

    // --------------------------------------------------
    // BYTE COMPLETE
    // --------------------------------------------------

    if (bitCount == 8)
    {
      if (frameBytes < FRAME_BYTES)
      {
        frameBuffer[frameBytes] =
          currentByte;

        frameBytes++;
      }

      currentByte = 0;
      bitCount = 0;
    }

    // --------------------------------------------------
    // EXACT FRAME COMPLETE
    // --------------------------------------------------

    if (
      frameSymbols == FRAME_BITS &&
      frameBytes == FRAME_BYTES &&
      bitCount == 0
    )
    {
      showCompleteFrame();
      return;
    }

    // --------------------------------------------------
    // SAFETY
    // --------------------------------------------------

    if (frameSymbols > FRAME_BITS)
    {
      resyncCount++;

      resetAssembler();
    }
  }

  // ====================================================
  // PROCESS RMT CHUNK
  // ====================================================

  void processChunk(
    rmt_item32_t *items,
    size_t count
  )
  {
    if (!items)
      return;

    chunksReceived++;

    /*
     * Do NOT reset frame state when a RingBuffer
     * packet ends.
     *
     * Example:
     *
     * chunk 1 = 40
     * chunk 2 = 27
     * chunk 3 = 53
     *
     * total = 120
     */

    for (
      size_t i = 0;
      i < count;
      i++
    )
    {
      processSymbol(items[i]);
    }
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
      frameBuffer,
      0,
      sizeof(frameBuffer)
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
      "Output: WLED REALTIME"
    );

    Serial.println(
      "RMT RX Channel: 6"
    );

    Serial.println(
      "RMT blocks: 2"
    );

    Serial.println(
      "Chunk assembler: ENABLED"
    );

    Serial.println(
      "Frame diagnostic: ENABLED"
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
      "Effects: OFF in realtime"
    );

    Serial.println(
      "Effects: AUTO RESTORE"
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
  // LOOP
  // ====================================================

  void loop() override
  {
    if (
      !rxReady ||
      rxRingBuffer == nullptr
    )
    {
      return;
    }

    // =================================================
    // RECEIVE ONE RMT CHUNK
    // =================================================

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

      processChunk(
        items,
        count
      );

      vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
      );
    }

    // =================================================
    // RETURN TO WLED EFFECTS
    // =================================================

    if (
      realtimeActive &&
      millis() - lastFrameTime >
      SIGNAL_TIMEOUT_MS
    )
    {
      exitRealtime();

      realtimeActive = false;

      resetAssembler();

      Serial.println(
        "Realtime OFF -> WLED effects"
      );
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
        "Chunks: "
      );

      Serial.print(
        chunksReceived
      );

      Serial.print(
        "  Frames: "
      );

      Serial.print(
        framesReceived
      );

      Serial.print(
        "  Shown: "
      );

      Serial.print(
        framesShown
      );

      Serial.print(
        "  Symbols: "
      );

      Serial.print(
        frameSymbols
      );

      Serial.print(
        "/"
      );

      Serial.print(
        FRAME_BITS
      );

      Serial.print(
        "  Bytes: "
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
        "  Resync: "
      );

      Serial.print(
        resyncCount
      );

      Serial.print(
        "  Mode: "
      );

      if (realtimeActive)
      {
        Serial.println(
          "REALTIME"
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
      "WLED realtime";

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

    info["chunks"] =
      chunksReceived;

    info["frames"] =
      framesReceived;

    info["shown"] =
      framesShown;

    info["resync"] =
      resyncCount;
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
