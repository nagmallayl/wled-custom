#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_6

// 80 MHz / 2 = 40 MHz
#define RMT_CLK_DIV 2

// ======================================================
// STRIP
// ======================================================

// 15 physical LEDs = 5 WS2811 ICs
#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)   // 120 bits

// ======================================================
// REAL MEASURED SIGNAL
// ======================================================
//
// 0: HIGH ~14-15 ticks
// 1: HIGH ~50-51 ticks
//
#define BIT_THRESHOLD 30

// ======================================================
// RMT FRAME SYNC
// ======================================================
//
// 3000 ticks × 25 ns = 75 us
//
// RMT ends the RX packet after this idle period.
// Therefore EACH ring-buffer packet is treated
// as one complete WS2811 frame.
//
// This is the critical change.
// ======================================================

#define RMT_IDLE_TICKS 3000

// Allow tiny variation around expected 120 symbols
#define MIN_FRAME_SYMBOLS 118
#define MAX_FRAME_SYMBOLS 122

// ======================================================
// REALTIME
// ======================================================

#define REALTIME_LOCK_MS   1000
#define SIGNAL_TIMEOUT_MS  1200


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;
  bool rxReady = false;

  // ====================================================
  // COMPLETE FRAME BUFFER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  bool realtimeActive = false;

  uint32_t lastFrameTime = 0;

  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t rxPackets       = 0;
  uint32_t goodFrames      = 0;
  uint32_t shownFrames     = 0;
  uint32_t badLengthFrames = 0;

  uint16_t lastSymbolCount = 0;

  uint32_t lastDebugTime = 0;


  // ====================================================
  // ENTER / REFRESH REALTIME
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
  // DECODE ONE COMPLETE RMT PACKET
  //
  // IMPORTANT:
  //
  // The RMT packet boundary itself is our frame reset.
  //
  // We DO NOT carry bitCount/frameBytes from one
  // packet into another.
  // ====================================================

  bool decodePacket(
    rmt_item32_t *items,
    size_t count
  )
  {
    if (!items)
      return false;

    lastSymbolCount = count;


    // ==================================================
    // FRAME LENGTH CHECK
    // ==================================================
    //
    // Expected = 120 WS2811 bits.
    //
    // We allow a very small margin in case RMT includes
    // an empty/end item.
    // ==================================================

    if (
      count < MIN_FRAME_SYMBOLS ||
      count > MAX_FRAME_SYMBOLS
    )
    {
      badLengthFrames++;
      return false;
    }


    memset(
      frameBuffer,
      0,
      sizeof(frameBuffer)
    );


    uint16_t validBits = 0;


    // ==================================================
    // DECODE FIRST 120 REAL SYMBOLS
    // ==================================================

    for (
      size_t i = 0;
      i < count && validBits < FRAME_BITS;
      i++
    )
    {
      uint16_t high = items[i].duration0;
      uint16_t low  = items[i].duration1;


      // Ignore only a completely empty/end item.
      if (
        high == 0 &&
        low == 0
      )
      {
        continue;
      }


      // =================================================
      // SAME DECODER THAT WORKED IN YOUR TEST
      // =================================================
      //
      // duration0:
      //
      // ~15 = bit 0
      // ~50 = bit 1
      //
      // No total H+L filtering.
      // No level reconstruction.
      // =================================================

      bool bit =
        high >= BIT_THRESHOLD;


      uint16_t byteIndex =
        validBits >> 3;


      uint8_t bitPosition =
        7 - (validBits & 0x07);


      if (bit)
      {
        frameBuffer[byteIndex] |=
          (1U << bitPosition);
      }


      validBits++;
    }


    // Must contain exactly our 120 bits.
    if (validBits != FRAME_BITS)
    {
      badLengthFrames++;
      return false;
    }


    return true;
  }


  // ====================================================
  // SHOW FRAME
  // ====================================================

  void showFrame()
  {
    // Stop normal WLED effects.
    keepRealtimeActive();


    // ==================================================
    // INPUT:
    //
    // G R B
    //
    // OUTPUT REQUIRED:
    //
    // B R G
    //
    // WLED setPixelColor() expects RGB arguments,
    // therefore:
    //
    // R = input B
    // G = input R
    // B = input G
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


    // ==================================================
    // REALTIME OUTPUT
    // ==================================================
    //
    // Same principle used by WLED realtime inputs:
    // write pixels, then show.
    // ==================================================

    strip.show();


    shownFrames++;

    lastFrameTime = millis();
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


    // 2 × 64 = 128 RMT items.
    // Our frame needs 120.
    config.mem_block_num =
      2;


    config.flags = 0;


    config.rx_config.filter_en =
      false;


    // This is now our REAL frame separator.
    config.rx_config.idle_threshold =
      RMT_IDLE_TICKS;


    esp_err_t err;


    err =
      rmt_config(&config);


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
      "WLED: v16.0.0"
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
      "Expected symbols: "
    );

    Serial.println(
      FRAME_BITS
    );

    Serial.println(
      "Frame sync: RMT IDLE BOUNDARY"
    );

    Serial.print(
      "Idle threshold: "
    );

    Serial.println(
      RMT_IDLE_TICKS
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
      "Input color: GRB"
    );

    Serial.println(
      "Output color: BRG"
    );

    Serial.println(
      "Effects: OFF during realtime"
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
      "Waiting for synchronized frames..."
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
    // RECEIVE ONE COMPLETE RMT FRAME
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


      rxPackets++;


      // =================================================
      // CRITICAL:
      //
      // Each ring-buffer packet starts fresh.
      // No decoder state carries over from an old frame.
      // =================================================

      if (
        decodePacket(
          items,
          count
        )
      )
      {
        goodFrames++;

        showFrame();
      }


      vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
      );
    }


    // =================================================
    // RETURN TO NORMAL WLED EFFECTS
    // =================================================

    if (
      realtimeActive &&
      millis() - lastFrameTime >
      SIGNAL_TIMEOUT_MS
    )
    {
      exitRealtime();

      realtimeActive = false;


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
        "Packets: "
      );

      Serial.print(
        rxPackets
      );


      Serial.print(
        "  Good: "
      );

      Serial.print(
        goodFrames
      );


      Serial.print(
        "  Shown: "
      );

      Serial.print(
        shownFrames
      );


      Serial.print(
        "  BadLen: "
      );

      Serial.print(
        badLengthFrames
      );


      Serial.print(
        "  Symbols: "
      );

      Serial.print(
        lastSymbolCount
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

    info["expected_symbols"] =
      FRAME_BITS;

    info["input_order"] =
      "GRB";

    info["output_order"] =
      "BRG";

    info["packets"] =
      rxPackets;

    info["good_frames"] =
      goodFrames;

    info["shown"] =
      shownFrames;

    info["bad_length"] =
      badLengthFrames;

    info["last_symbols"] =
      lastSymbolCount;
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
