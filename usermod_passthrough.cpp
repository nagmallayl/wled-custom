#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// HARDWARE
// ======================================================

#define INPUT_GPIO 25

// Keep RX away from WLED low-numbered RMT channels
#define RX_CHANNEL RMT_CHANNEL_6

// ESP32:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2


// ======================================================
// WS2811 TEST STRIP
// ======================================================

#define PHYSICAL_LEDS 15
#define WS2811_ICS    5

#define FRAME_BYTES   (WS2811_ICS * 3)
#define FRAME_BITS    (FRAME_BYTES * 8)


// ======================================================
// MEASURED INPUT TIMING
// ======================================================
//
// Measured from GPIO25:
//
// Bit 0:
// HIGH = 14~15
// LOW  = 41~42
//
// Bit 1:
// HIGH = 50~51
// LOW  = 24~25
//
// Decode based mainly on HIGH duration.
// ======================================================

#define BIT_ONE_THRESHOLD 30

#define VALID_HIGH_MIN 5
#define VALID_HIGH_MAX 80

// L=84 was seen inside data.
// Real reset must be much longer.
#define RESET_LOW_TICKS 1000


// ======================================================
// AUTO PASSTHROUGH
// ======================================================
//
// Increased from 300 ms to 2000 ms.
//
// This prevents brief RX gaps from letting WLED effects
// appear between passthrough frames.
// ======================================================

#define SIGNAL_TIMEOUT_MS 2000


// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;


  // ====================================================
  // DECODER STATE
  // ====================================================

  uint8_t rxFrame[FRAME_BYTES];

  uint8_t currentByte = 0;
  uint8_t bitCount    = 0;
  uint8_t frameBytes  = 0;


  // ====================================================
  // RMT LEVEL-AWARE DECODER STATE
  // ====================================================

  bool waitingForLow = false;

  uint16_t pendingHigh = 0;


  // ====================================================
  // LATEST COMPLETE FRAME
  // ====================================================

  uint8_t latestFrame[FRAME_BYTES];

  bool haveFrame = false;

  bool passthroughActive = false;


  // ====================================================
  // FRAME SEQUENCE
  // ====================================================

  uint32_t frameSequence = 0;

  uint32_t lastAppliedSequence = 0;


  // ====================================================
  // STATISTICS
  // ====================================================

  uint32_t framesReceived = 0;

  uint32_t framesApplied = 0;

  uint32_t invalidPulses = 0;

  uint32_t resetsSeen = 0;

  uint32_t resyncCount = 0;

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

    pendingHigh = 0;

    waitingForLow = false;
  }


  // ====================================================
  // COMMIT COMPLETE FRAME
  // ====================================================

  void commitFrame()
  {
    if (
        frameBytes != FRAME_BYTES ||
        bitCount != 0
    ) {
      return;
    }


    // --------------------------------------------------
    // Store newest complete frame
    // --------------------------------------------------

    memcpy(
        latestFrame,
        rxFrame,
        FRAME_BYTES
    );


    haveFrame = true;

    passthroughActive = true;


    framesReceived++;

    frameSequence++;


    lastFrameTime = millis();


    // --------------------------------------------------
    // Immediately prepare for next incoming frame
    // --------------------------------------------------

    currentByte = 0;

    bitCount = 0;

    frameBytes = 0;

    pendingHigh = 0;

    waitingForLow = false;


    // ==================================================
    // IMPORTANT:
    //
    // NO strip.trigger()
    //
    // WLED will continue its normal rendering cycle.
    // handleOverlayDraw() will overwrite the effect
    // immediately before every LED output.
    // ==================================================
  }


  // ====================================================
  // DECODE ONE COMPLETE HIGH + LOW PULSE
  // ====================================================

  void decodePulse(
      uint16_t high,
      uint16_t low
  )
  {
    // --------------------------------------------------
    // Validate HIGH
    // --------------------------------------------------

    if (
        high < VALID_HIGH_MIN ||
        high > VALID_HIGH_MAX
    ) {

      invalidPulses++;

      return;
    }


    // --------------------------------------------------
    // Decode bit using measured HIGH duration
    // --------------------------------------------------

    const bool bit =
        high >= BIT_ONE_THRESHOLD;


    currentByte <<= 1;


    if (bit) {
      currentByte |= 1;
    }


    bitCount++;


    // --------------------------------------------------
    // Complete byte
    // --------------------------------------------------

    if (
        bitCount == 8
    ) {

      if (
          frameBytes < FRAME_BYTES
      ) {

        rxFrame[frameBytes] =
            currentByte;

        frameBytes++;
      }


      currentByte = 0;

      bitCount = 0;
    }


    // --------------------------------------------------
    // Complete fixed frame
    //
    // 5 IC × 24 bits = 120 bits
    // --------------------------------------------------

    if (
        frameBytes == FRAME_BYTES &&
        bitCount == 0
    ) {

      commitFrame();
    }
  }


  // ====================================================
  // PROCESS ONE RMT SEGMENT
  // ====================================================

  void processSegment(
      uint8_t level,
      uint16_t duration
  )
  {
    if (
        duration == 0
    ) {
      return;
    }


    // =================================================
    // LOW LEVEL
    // =================================================

    if (
        level == 0
    ) {

      // ------------------------------------------------
      // TRUE WS2811 RESET
      // ------------------------------------------------

      if (
          duration >= RESET_LOW_TICKS
      ) {

        resetsSeen++;


        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        ) {

          commitFrame();

        }
        else if (
            frameBytes != 0 ||
            bitCount != 0
        ) {

          resyncCount++;


          currentByte = 0;

          bitCount = 0;

          frameBytes = 0;
        }


        waitingForLow = false;

        pendingHigh = 0;


        return;
      }


      // ------------------------------------------------
      // LOW following a HIGH = complete WS2811 bit
      // ------------------------------------------------

      if (
          waitingForLow
      ) {

        decodePulse(
            pendingHigh,
            duration
        );


        waitingForLow = false;

        pendingHigh = 0;
      }


      /*
       * LOW without a preceding HIGH can happen at
       * an RMT chunk boundary.
       *
       * Ignore it instead of resetting the decoder.
       */

      return;
    }


    // =================================================
    // HIGH LEVEL
    // =================================================

    if (
        level == 1
    ) {

      /*
       * HIGH begins a new WS2811 bit.
       */

      pendingHigh = duration;

      waitingForLow = true;


      return;
    }
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
    ) {

      // ------------------------------------------------
      // Respect actual electrical levels.
      // ------------------------------------------------

      processSegment(
          items[i].level0,
          items[i].duration0
      );


      processSegment(
          items[i].level1,
          items[i].duration1
      );
    }
  }


  // ====================================================
  // APPLY LATEST FRAME TO WLED
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
    ) {

      const uint8_t p =
          i * 3;


      // Incoming:
      //
      // G R B

      const uint8_t inputG =
          latestFrame[p + 0];


      const uint8_t inputR =
          latestFrame[p + 1];


      const uint8_t inputB =
          latestFrame[p + 2];


      // =================================================
      // COLOR CONVERSION
      //
      // GRB -> BRG
      //
      // WLED RGB arguments:
      //
      // R = input B
      // G = input R
      // B = input G
      // =================================================

      const uint8_t outputR =
          inputB;


      const uint8_t outputG =
          inputR;


      const uint8_t outputB =
          inputG;


      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    // ==================================================
    // IMPORTANT:
    //
    // NO strip.show()
    //
    // WLED itself performs the actual output after
    // handleOverlayDraw().
    // ==================================================


    if (
        lastAppliedSequence != frameSequence
    ) {

      lastAppliedSequence =
          frameSequence;

      framesApplied++;
    }
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


    // --------------------------------------------------
    // 2 blocks = 128 RMT items
    //
    // Frame = 120 WS2811 bits
    // --------------------------------------------------

    config.mem_block_num =
        2;


    config.flags =
        0;


    config.rx_config.filter_en =
        false;


    // --------------------------------------------------
    // Keep the setting that previously captured frames.
    // --------------------------------------------------

    config.rx_config.idle_threshold =
        3000;


    esp_err_t err;


    // --------------------------------------------------
    // Configure RMT
    // --------------------------------------------------

    err =
        rmt_config(
            &config
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT CONFIG ERROR: "
      );

      Serial.println(
          err
      );


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


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT DRIVER ERROR: "
      );

      Serial.println(
          err
      );


      return false;
    }


    // --------------------------------------------------
    // Get ring buffer
    // --------------------------------------------------

    err =
        rmt_get_ringbuf_handle(
            RX_CHANNEL,
            &rxRingBuffer
        );


    if (
        err != ESP_OK ||
        rxRingBuffer == nullptr
    ) {

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


    if (
        err != ESP_OK
    ) {

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
        "RMT level-aware decoder: ENABLED"
    );


    Serial.println(
        "Protocol: WS2811"
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
        "Measured 0: H14~15"
    );


    Serial.println(
        "Measured 1: H50~51"
    );


    Serial.println(
        "Bit threshold: H >= 30"
    );


    Serial.println(
        "Input: GRB"
    );


    Serial.println(
        "Output: BRG"
    );


    Serial.println(
        "Direct strip.show(): DISABLED"
    );


    Serial.println(
        "strip.trigger(): DISABLED"
    );


    Serial.print(
        "Signal timeout: "
    );

    Serial.print(
        SIGNAL_TIMEOUT_MS
    );

    Serial.println(
        " ms"
    );


    Serial.println(
        "Output: WLED OVERLAY"
    );


    Serial.println(
        "================================"
    );


    if (
        setupRX()
    ) {

      rxReady = true;


      Serial.println(
          "RMT RX READY"
      );

    } else {

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
    /*
     * WLED effects may render normally.
     *
     * While passthrough is active, we overwrite them
     * with the latest GPIO25 frame immediately before
     * WLED performs its physical LED output.
     */

    if (
        passthroughActive &&
        haveFrame
    ) {

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


    // --------------------------------------------------
    // NON-BLOCKING RX
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


    if (items) {

      const size_t count =
          receivedSize /
          sizeof(rmt_item32_t);


      processSymbols(
          items,
          count
      );


      // Always release RMT memory.

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // =================================================
    // AUTO RETURN TO NORMAL WLED
    // =================================================

    if (
        passthroughActive
    ) {

      const uint32_t now =
          millis();


      if (
          now - lastFrameTime >
          SIGNAL_TIMEOUT_MS
      ) {

        passthroughActive =
            false;


        haveFrame =
            false;


        resetDecoder();


        Serial.println(
            "Passthrough OFF -> WLED"
        );
      }
    }


    // =================================================
    // DEBUG
    // =================================================

    const uint32_t now =
        millis();


    if (
        now - lastDebugTime >= 2000
    ) {

      lastDebugTime =
          now;


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
          "  Invalid: "
      );

      Serial.print(
          invalidPulses
      );


      Serial.print(
          "  Reset: "
      );

      Serial.print(
          resetsSeen
      );


      Serial.print(
          "  Resync: "
      );

      Serial.print(
          resyncCount
      );


      Serial.print(
          "  RX Bytes: "
      );

      Serial.print(
          frameBytes
      );


      Serial.print(
          "  Mode: "
      );


      if (
          passthroughActive
      ) {

        Serial.println(
            "PASSTHROUGH"
        );

      } else {

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


    info["timeout_ms"] =
        SIGNAL_TIMEOUT_MS;


    info["frames"] =
        framesReceived;


    info["applied"] =
        framesApplied;


    info["invalid"] =
        invalidPulses;


    info["reset"] =
        resetsSeen;


    info["resync"] =
        resyncCount;
  }


  uint16_t getId() override
  {
    return 0x5041;
  }
};


// ======================================================
// REGISTER USERMOD
// ======================================================

static PassthroughUsermod passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
