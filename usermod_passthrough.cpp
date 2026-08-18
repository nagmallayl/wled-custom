#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// INPUT CONFIG
// ======================================================

#define INPUT_GPIO 25

// RMT is now used ONLY for RX.
// WLED output is configured in UI as I2S on GPIO16.
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32 classic:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2


// ======================================================
// WS2811 FRAME
// ======================================================

// Short test strip:
// 15 physical LEDs
// 5 WS2811 ICs
#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)


// ======================================================
// MEASURED INPUT TIMING
// ======================================================
//
// From your actual GPIO25 diagnostic:
//
// 0:
// HIGH ≈ 14~15 ticks
//
// 1:
// HIGH ≈ 50~51 ticks
//
// Therefore:
#define BIT_THRESHOLD 30


// ======================================================
// RMT RX
// ======================================================
//
// Keep 3000 because RX-only became stable with it.
#define RMT_IDLE_TICKS 3000


// ======================================================
// REALTIME MODE
// ======================================================

#define REALTIME_LOCK_MS 1000

// If no valid frame arrives for this long,
// give control back to normal WLED effects.
#define SIGNAL_TIMEOUT_MS 1500


// ======================================================
// DEBUG
// ======================================================

#define FRAME_DEBUG_INTERVAL_MS 2000


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;


  // ====================================================
  // FRAME BUFFER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];


  // ====================================================
  // REALTIME STATE
  // ====================================================

  bool realtimeActive = false;

  uint32_t lastFrameTime = 0;


  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t packetsReceived = 0;

  uint32_t goodFrames = 0;

  uint32_t shownFrames = 0;

  uint32_t shortPackets = 0;

  uint32_t longPackets = 0;

  uint16_t lastPacketSymbols = 0;

  uint32_t lastDebugTime = 0;

  uint32_t lastFrameDebugTime = 0;


  // ====================================================
  // REALTIME
  // ====================================================

  void keepRealtimeActive()
  {
    /*
     * REALTIME_MODE_UDP:
     *
     * Enables official WLED realtime mode,
     * preventing the normal effect engine
     * from fighting our passthrough.
     *
     * It does NOT automatically call show(),
     * so we control exactly when the I2S output occurs.
     */

    realtimeLock(
      REALTIME_LOCK_MS,
      REALTIME_MODE_UDP
    );

    realtimeActive = true;
  }


  // ====================================================
  // FRAME DIAGNOSTIC
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
      "========== PASSTHROUGH FRAME =========="
    );

    for (
      uint8_t i = 0;
      i < WS2811_ICS;
      i++
    )
    {
      uint8_t p = i * 3;

      Serial.print("IC");
      Serial.print(i + 1);

      Serial.print(": G=");
      Serial.print(
        frameBuffer[p + 0]
      );

      Serial.print(" R=");
      Serial.print(
        frameBuffer[p + 1]
      );

      Serial.print(" B=");
      Serial.println(
        frameBuffer[p + 2]
      );
    }

    Serial.println(
      "======================================="
    );
  }


  // ====================================================
  // DECODE ONE RECEIVED PACKET
  // ====================================================

  bool decodePacket(
    rmt_item32_t *items,
    size_t count
  )
  {
    if (!items)
      return false;


    packetsReceived++;

    lastPacketSymbols =
      count;


    // --------------------------------------------------
    // Ignore incomplete RX packets.
    //
    // This is exactly what became stable in RX-only:
    // only full 120-symbol packets are accepted.
    // --------------------------------------------------

    if (
      count < FRAME_BITS
    )
    {
      shortPackets++;

      return false;
    }


    if (
      count > FRAME_BITS + 8
    )
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
      i < count &&
      bitIndex < FRAME_BITS;
      i++
    )
    {
      if (
        items[i].duration0 == 0 &&
        items[i].duration1 == 0
      )
      {
        continue;
      }


      /*
       * Use the exact decoder that produced
       * stable identical IC1...IC5 frames in RX-only.
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


    if (
      bitIndex != FRAME_BITS
    )
    {
      shortPackets++;

      return false;
    }


    goodFrames++;


    return true;
  }


  // ====================================================
  // OUTPUT COMPLETE FRAME THROUGH WLED / I2S
  // ====================================================

  void showFrame()
  {
    // --------------------------------------------------
    // Stop normal WLED effects.
    // --------------------------------------------------

    keepRealtimeActive();


    // ==================================================
    // COLOR MAPPING
    //
    // Incoming bytes:
    //
    // G R B
    //
    // Required output:
    //
    // B R G
    //
    // WLED setPixelColor() arguments are R,G,B:
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
    // WLED LED Preferences is now:
    //
    // GPIO16
    // Driver = I2S
    //
    // Therefore strip.show() sends through I2S,
    // NOT through our RMT RX peripheral.
    // ==================================================

    strip.show();


    shownFrames++;

    lastFrameTime =
      millis();


    printFrameDiagnostic();
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
      (gpio_num_t)
      INPUT_GPIO;


    config.clk_div =
      RMT_CLK_DIV;


    /*
     * Keep exactly the RX configuration
     * that worked in RX-only.
     */

    config.mem_block_num =
      2;


    config.flags =
      0;


    config.rx_config.filter_en =
      false;


    config.rx_config.idle_threshold =
      RMT_IDLE_TICKS;


    esp_err_t err;


    err =
      rmt_config(
        &config
      );


    if (
      err != ESP_OK
    )
    {
      Serial.print(
        "RMT CONFIG ERROR: "
      );

      Serial.println(
        err
      );

      return false;
    }


    err =
      rmt_driver_install(
        RX_CHANNEL,
        4096,
        0
      );


    if (
      err != ESP_OK
    )
    {
      Serial.print(
        "RMT DRIVER ERROR: "
      );

      Serial.println(
        err
      );

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


    if (
      err != ESP_OK
    )
    {
      Serial.print(
        "RMT RX START ERROR: "
      );

      Serial.println(
        err
      );

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
      "WS2811 PASSTHROUGH - RMT RX / I2S TX"
    );


    Serial.print(
      "Input GPIO: "
    );

    Serial.println(
      INPUT_GPIO
    );


    Serial.println(
      "RX Driver: RMT"
    );


    Serial.println(
      "RMT RX Channel: 6"
    );


    Serial.println(
      "RMT blocks: 2"
    );


    Serial.print(
      "RMT idle threshold: "
    );

    Serial.println(
      RMT_IDLE_TICKS
    );


    Serial.println(
      "Output GPIO: WLED configured GPIO16"
    );


    Serial.println(
      "Output Driver: I2S"
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
      "Input order: GRB"
    );


    Serial.println(
      "Output order: BRG"
    );


    Serial.println(
      "WLED realtime: ENABLED"
    );


    Serial.println(
      "Effects: suspended during passthrough"
    );


    Serial.println(
      "Effects: auto restore on signal loss"
    );


    Serial.println(
      "================================"
    );


    if (
      setupRX()
    )
    {
      rxReady =
        true;


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
    // RECEIVE RMT PACKET
    // =================================================

    size_t receivedSize =
      0;


    rmt_item32_t *items =
      (rmt_item32_t *)
      xRingbufferReceive(
        rxRingBuffer,
        &receivedSize,
        0
      );


    if (
      items
    )
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
    // RETURN TO NORMAL WLED EFFECTS
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
        "Realtime OFF -> WLED effects"
      );
    }


    // =================================================
    // DEBUG
    // =================================================

    uint32_t now =
      millis();


    if (
      now - lastDebugTime >=
      2000
    )
    {
      lastDebugTime =
        now;


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


      if (
        realtimeActive
      )
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
        "WS2811 Passthrough"
      );


    info["input_gpio"] =
      INPUT_GPIO;


    info["rx_driver"] =
      "RMT";


    info["rmt_channel"] =
      6;


    info["output_gpio"] =
      16;


    info["tx_driver"] =
      "I2S";


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


    info["good"] =
      goodFrames;


    info["shown"] =
      shownFrames;


    info["short"] =
      shortPackets;


    info["long"] =
      longPackets;
  }


  uint16_t getId() override
  {
    return 0x5044;
  }
};


static PassthroughUsermod
passthroughUsermod;


REGISTER_USERMOD(
  passthroughUsermod
);
