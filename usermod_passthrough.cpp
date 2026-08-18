#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32: 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2

// ======================================================
// STRIP
// ======================================================

#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)   // 120

// ======================================================
// MEASURED WS2811 TIMING
// ======================================================
//
// bit 0:
// HIGH ≈ 14~15 ticks
//
// bit 1:
// HIGH ≈ 50~51 ticks
//
#define BIT_THRESHOLD 30

// ======================================================
// FRAME RESET / SYNC
// ======================================================
//
// Old value:
// 3000 ticks = 75 us
//
// New value:
// 1000 ticks = 25 us
//
// Normal data LOW previously measured as high as ~84
// ticks only (~2.1 us), so 1000 still leaves a very
// large margin.
//
// The purpose is for RMT RX itself to terminate at the
// WS2811 reset gap and give us a naturally synchronized
// frame packet.
//
#define RMT_IDLE_TICKS 1000

// ======================================================
// REALTIME
// ======================================================

#define REALTIME_LOCK_MS   1000
#define SIGNAL_TIMEOUT_MS  1500

// Diagnostic print interval
#define FRAME_DEBUG_INTERVAL_MS 1000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;
  bool rxReady = false;

  // ====================================================
  // FRAME BUFFER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  bool realtimeActive = false;

  uint32_t lastFrameTime = 0;

  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t packetsReceived = 0;
  uint32_t goodFrames      = 0;
  uint32_t shownFrames     = 0;
  uint32_t shortPackets    = 0;
  uint32_t longPackets     = 0;

  uint16_t lastPacketSymbols = 0;

  uint32_t lastDebugTime = 0;
  uint32_t lastFrameDebugTime = 0;


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
  // PRINT FRAME DATA
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
      "========== FRAME DATA =========="
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
      "================================"
    );
  }


  // ====================================================
  // DECODE SYNCHRONIZED PACKET
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

    /*
     * Because the RMT hardware now terminates reception
     * at the reset/idle gap, THIS packet should begin at
     * the beginning of a new WS2811 frame.
     *
     * We therefore NEVER carry decoder state from one
     * packet to the next.
     */

    if (count < FRAME_BITS)
    {
      shortPackets++;
      return false;
    }

    /*
     * More than 120 can happen if an end marker or extra
     * RMT item is present. We only decode the first 120
     * non-empty items.
     */

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
      // Ignore only completely empty RMT items.
      if (
        items[i].duration0 == 0 &&
        items[i].duration1 == 0
      ) {
        continue;
      }

      /*
       * Use the decoder that actually worked with your
       * measured signal.
       *
       * duration0 < 30  -> 0
       * duration0 >= 30 -> 1
       */

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
  // SHOW FRAME
  // ====================================================

  void showFrame()
  {
    keepRealtimeActive();

    // ==================================================
    // INPUT:
    // G R B
    //
    // REQUIRED OUTPUT:
    // B R G
    //
    // WLED API receives R,G,B parameters:
    //
    // output R = input B
    // output G = input R
    // output B = input G
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

      strip.setPixelColor(
        i,
        inputB,  // R
        inputR,  // G
        inputG   // B
      );
    }

    strip.show();

    shownFrames++;

    lastFrameTime = millis();

    printFrameDiagnostic();
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

    /*
     * 2 × 64 = 128 RMT items.
     *
     * Enough for our 120-bit frame.
     */
    config.mem_block_num =
      2;

    config.flags =
      0;

    config.rx_config.filter_en =
      false;

    /*
     * CRITICAL CHANGE:
     *
     * 1000 instead of 3000.
     *
     * The hardware reset/idle boundary should now end
     * each RX transaction before the next WS2811 frame.
     */
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
      "Frame sync: HARDWARE IDLE RESET"
    );

    Serial.print(
      "RMT idle threshold: "
    );

    Serial.print(
      RMT_IDLE_TICKS
    );

    Serial.println(
      " ticks"
    );

    Serial.println(
      "RMT idle time: 25 us"
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
      "Effects: OFF during realtime"
    );

    Serial.println(
      "Effects: AUTO RESTORE"
    );

    Serial.println(
      "Frame diagnostic: ENABLED"
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
    // RECEIVE ONE RESET-SYNCHRONIZED PACKET
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
    // RETURN TO NORMAL WLED
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
        packetsReceived
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

      Serial.print(
        lastPacketSymbols
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

    info["idle_ticks"] =
      RMT_IDLE_TICKS;

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

    info["packets"] =
      packetsReceived;

    info["good_frames"] =
      goodFrames;

    info["shown"] =
      shownFrames;

    info["short_packets"] =
      shortPackets;

    info["long_packets"] =
      longPackets;
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
