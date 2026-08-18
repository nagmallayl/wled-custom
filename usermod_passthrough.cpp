#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIG
// ======================================================

#define INPUT_GPIO 25

// RX channel kept away from low channels
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32 classic
// 80 MHz / 2 = 40 MHz
#define RMT_CLK_DIV 2


// ======================================================
// WS2811
// ======================================================

// Current short test strip:
// 15 physical LEDs
// 5 WS2811 ICs
#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)


// ======================================================
// MEASURED TIMING
// ======================================================
//
// Actual GPIO25 measurements:
//
// bit 0:
// H = 14~15
//
// bit 1:
// H = 50~51
//
// Therefore:
//
// H < 30  = 0
// H >= 30 = 1
//
// This is the decoder that previously gave us
// stable Frames > 1000.
// ======================================================

#define BIT_THRESHOLD 30


// ======================================================
// REALTIME
// ======================================================

// Refresh realtime timeout with every valid frame.
#define REALTIME_LOCK_MS 1000

// If input completely disappears,
// explicitly return control to normal WLED.
#define SIGNAL_TIMEOUT_MS 1200


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;


  // ====================================================
  // DECODER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  uint8_t currentByte = 0;
  uint8_t bitCount    = 0;
  uint8_t frameBytes  = 0;


  // ====================================================
  // STATE
  // ====================================================

  bool realtimeActive = false;

  uint32_t lastFrameTime = 0;


  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;
  uint32_t framesShown    = 0;

  uint32_t lastDebugTime  = 0;


  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    currentByte = 0;
    bitCount    = 0;
    frameBytes  = 0;
  }


  // ====================================================
  // ENTER / REFRESH WLED REALTIME MODE
  // ====================================================

  void keepRealtimeActive()
  {
    /*
     * REALTIME_MODE_UDP is intentional.
     *
     * In WLED 16.0.0:
     *
     * REALTIME_MODE_GENERIC causes realtimeLock()
     * itself to call strip.show().
     *
     * UDP mode does not.
     *
     * We want exactly ONE show after putting our
     * pixels into WLED.
     */

    realtimeLock(
        REALTIME_LOCK_MS,
        REALTIME_MODE_UDP
    );

    realtimeActive = true;
  }


  // ====================================================
  // SEND COMPLETE FRAME THROUGH WLED
  // ====================================================

  void showFrame()
  {
    /*
     * First put WLED into its official realtime mode.
     *
     * This prevents normal effects from running
     * against our passthrough data.
     */

    keepRealtimeActive();


    // ==================================================
    // COLOR CONVERSION
    //
    // Incoming:
    //
    // G R B
    //
    // Required:
    //
    // B R G
    //
    // WLED setPixelColor() arguments are RGB, therefore:
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
     * This show() is now intentional.
     *
     * The difference from our earlier tests:
     *
     * WLED effects are blocked by realtimeMode,
     * so strip.show() is no longer fighting
     * strip.service().
     */

    strip.show();


    framesShown++;

    lastFrameTime = millis();
  }


  // ====================================================
  // COMPLETE FRAME
  // ====================================================

  void commitFrame()
  {
    if (
        frameBytes != FRAME_BYTES ||
        bitCount != 0
    )
    {
      resetDecoder();
      return;
    }


    framesReceived++;


    /*
     * Output immediately.
     *
     * We don't wait for handleOverlayDraw().
     */

    showFrame();


    /*
     * Start receiving next frame.
     */

    resetDecoder();
  }


  // ====================================================
  // PROCESS RMT ITEMS
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
      /*
       * This intentionally matches the decoder
       * that worked best in your actual tests.
       *
       * We use duration0 only.
       *
       * No H+L validation.
       * No level0/level1 reconstruction.
       */

      uint16_t high =
          items[i].duration0;


      // Completely empty RMT item

      if (
          items[i].duration0 == 0 &&
          items[i].duration1 == 0
      )
      {
        continue;
      }


      // ----------------------------------------------
      // Decode using measured threshold
      // ----------------------------------------------

      bool bit =
          high >= BIT_THRESHOLD;


      currentByte <<= 1;


      if (bit)
      {
        currentByte |= 1;
      }


      bitCount++;


      // ----------------------------------------------
      // Complete byte
      // ----------------------------------------------

      if (
          bitCount == 8
      )
      {
        if (
            frameBytes < FRAME_BYTES
        )
        {
          frameBuffer[frameBytes] =
              currentByte;

          frameBytes++;
        }


        currentByte = 0;
        bitCount = 0;
      }


      // ----------------------------------------------
      // Complete fixed-length WS2811 frame
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
  // SETUP RMT
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
     * Keep the exact receive configuration
     * that produced stable Frames previously.
     */

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


    if (
        err != ESP_OK
    )
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


    if (
        err != ESP_OK
    )
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


    if (
        err != ESP_OK
    )
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
        "WLED realtime mode: UDP"
    );


    Serial.println(
        "Effects during passthrough: BLOCKED"
    );


    Serial.println(
        "Direct realtime strip.show(): ENABLED"
    );


    Serial.println(
        "handleOverlayDraw(): NOT USED"
    );


    Serial.println(
        "================================"
    );


    if (
        setupRX()
    )
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


    // --------------------------------------------------
    // NON-BLOCKING RMT RECEIVE
    // --------------------------------------------------

    size_t receivedSize =
        0;


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
    // RETURN TO NORMAL WLED
    // =================================================

    if (
        realtimeActive &&
        millis() - lastFrameTime >
        SIGNAL_TIMEOUT_MS
    )
    {
      /*
       * Official WLED realtime exit.
       *
       * Restores normal brightness/state and
       * resumes the normal effects engine.
       */

      exitRealtime();


      realtimeActive = false;

      resetDecoder();


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
      lastDebugTime =
          now;


      Serial.print(
          "Frames: "
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


    info["realtime"] =
        realtimeActive;


    info["frames"] =
        framesReceived;


    info["shown"] =
        framesShown;
  }


  uint16_t getId() override
  {
    return 0x5041;
  }
};


// ======================================================
// REGISTER
// ======================================================

static PassthroughUsermod
passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
