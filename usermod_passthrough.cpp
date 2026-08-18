#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// WS2811 PASSTHROUGH - FINAL
//
// INPUT : GPIO25 / RMT RX
// OUTPUT: WLED / I2S / GPIO16
//
// WLED : v16.0.0
// ======================================================


// ======================================================
// INPUT
// ======================================================

#define INPUT_GPIO 25

// RMT is used ONLY for receiving.
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2


// ======================================================
// STRIP
// ======================================================

// Current strip:
// 15 physical LEDs
// 5 WS2811 ICs
#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)


// ======================================================
// MEASURED WS2811 INPUT TIMING
// ======================================================
//
// Actual measurements:
//
// bit 0:
// HIGH ≈ 14~15 ticks
//
// bit 1:
// HIGH ≈ 50~51 ticks
//
// Therefore:
#define BIT_THRESHOLD 30


// ======================================================
// RMT RX
// ======================================================

// This value produced clean 120-symbol frames
// after separating WLED output to I2S.
#define RMT_IDLE_TICKS 3000


// ======================================================
// WLED REALTIME
// ======================================================

// Realtime is refreshed for every valid frame.
#define REALTIME_LOCK_MS 1000

// Return control to normal WLED effects if
// GPIO25 signal disappears.
#define SIGNAL_TIMEOUT_MS 1500


// ======================================================
// DEBUG
// ======================================================

// Lightweight status only every 10 seconds.
#define DEBUG_INTERVAL_MS 10000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  bool realtimeActive = false;


  // ====================================================
  // FRAME
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];


  // ====================================================
  // TIMING
  // ====================================================

  uint32_t lastFrameTime = 0;
  uint32_t lastDebugTime = 0;


  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t packetsReceived = 0;
  uint32_t goodFrames      = 0;
  uint32_t shownFrames     = 0;

  uint32_t shortPackets    = 0;
  uint32_t longPackets     = 0;


  // ====================================================
  // WLED REALTIME
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
  // DECODE COMPLETE RMT PACKET
  // ====================================================

  bool decodePacket(
    rmt_item32_t *items,
    size_t count
  )
  {
    if (!items)
      return false;


    packetsReceived++;


    // --------------------------------------------------
    // We only accept complete WS2811 frames.
    //
    // Expected:
    // 5 IC × 24 bit = 120 symbols
    // --------------------------------------------------

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


    // ==================================================
    // DECODE FIRST 120 NON-EMPTY SYMBOLS
    // ==================================================

    for (
      size_t i = 0;
      i < count && bitIndex < FRAME_BITS;
      i++
    )
    {
      // Ignore an empty RMT end item.
      if (
        items[i].duration0 == 0 &&
        items[i].duration1 == 0
      )
      {
        continue;
      }


      // ------------------------------------------------
      // Decoder confirmed by actual measurements:
      //
      // H < 30  = 0
      // H >= 30 = 1
      // ------------------------------------------------

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
  // OUTPUT FRAME
  // ====================================================

  void showFrame()
  {
    // --------------------------------------------------
    // Enter/refresh WLED realtime mode.
    //
    // Normal WLED effects are suspended while the
    // passthrough signal is active.
    // --------------------------------------------------

    keepRealtimeActive();


    // ==================================================
    // COLOR MAPPING
    //
    // Incoming:
    //
    // G R B
    //
    // Required output:
    //
    // B R G
    //
    // WLED setPixelColor() expects:
    //
    // R, G, B
    //
    // Therefore:
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
      uint8_t p =
        i * 3;


      uint8_t inputG =
        frameBuffer[p + 0];


      uint8_t inputR =
        frameBuffer[p + 1];


      uint8_t inputB =
        frameBuffer[p + 2];


      strip.setPixelColor(
        i,
        inputB,  // output R
        inputR,  // output G
        inputG   // output B
      );
    }


    // ==================================================
    // IMPORTANT
    //
    // WLED LED Preferences must remain:
    //
    // GPIO16
    // Driver = I2S
    //
    // Therefore this show() uses I2S output and does
    // NOT interfere with RMT RX on GPIO25.
    // ==================================================

    strip.show();


    shownFrames++;


    lastFrameTime =
      millis();
  }


  // ====================================================
  // RMT RX SETUP
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


    // Two blocks proved sufficient for the
    // 120-symbol frame.
    config.mem_block_num =
      2;


    config.flags =
      0;


    config.rx_config.filter_en =
      false;


    config.rx_config.idle_threshold =
      RMT_IDLE_TICKS;


    esp_err_t err;


    // --------------------------------------------------
    // Configure RMT
    // --------------------------------------------------

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


    // --------------------------------------------------
    // Install RX driver
    // --------------------------------------------------

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


    // --------------------------------------------------
    // Get RX ring buffer
    // --------------------------------------------------

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


    // --------------------------------------------------
    // Start RX
    // --------------------------------------------------

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
      "WS2811 PASSTHROUGH FINAL"
    );


    Serial.println(
      "WLED: v16.0.0"
    );


    Serial.print(
      "Input GPIO: "
    );

    Serial.println(
      INPUT_GPIO
    );


    Serial.println(
      "RX: RMT Channel 6"
    );


    Serial.println(
      "Output: WLED I2S"
    );


    Serial.println(
      "Output GPIO: 16"
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
      "Input order: GRB"
    );


    Serial.println(
      "Output order: BRG"
    );


    Serial.println(
      "Effects: AUTO"
    );


    Serial.println(
      "Passthrough: REALTIME"
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
    // RECEIVE
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


      if (
        decodePacket(
          items,
          count
        )
      )
      {
        showFrame();
      }


      vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
      );
    }


    // =================================================
    // SIGNAL LOST
    //
    // Return automatically to normal WLED effects.
    // =================================================

    if (
      realtimeActive &&
      millis() - lastFrameTime >
      SIGNAL_TIMEOUT_MS
    )
    {
      exitRealtime();


      realtimeActive =
        false;


      Serial.println(
        "Passthrough OFF -> WLED"
      );
    }


    // =================================================
    // LIGHTWEIGHT DEBUG
    // =================================================

    uint32_t now =
      millis();


    if (
      now - lastDebugTime >=
      DEBUG_INTERVAL_MS
    )
    {
      lastDebugTime = now;


      Serial.print(
        "Frames: "
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
        "  Mode: "
      );


      if (realtimeActive)
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
  // WLED INFO
  // ====================================================

  void addToJsonInfo(
    JsonObject &root
  ) override
  {
    JsonObject info =
      root["u"]
      .createNestedObject(
        "WS2811 Passthrough"
      );


    info["input"] =
      "GPIO25 / RMT";


    info["output"] =
      "GPIO16 / I2S";


    info["physical_leds"] =
      PHYSICAL_LEDS;


    info["ws2811_ics"] =
      WS2811_ICS;


    info["input_order"] =
      "GRB";


    info["output_order"] =
      "BRG";


    info["frames"] =
      goodFrames;


    info["shown"] =
      shownFrames;


    info["short"] =
      shortPackets;


    info["long"] =
      longPackets;


    info["active"] =
      realtimeActive;
  }


  uint16_t getId() override
  {
    return 0x5044;
  }
};


static PassthroughUsermod passthroughUsermod;


REGISTER_USERMOD(
  passthroughUsermod
);
