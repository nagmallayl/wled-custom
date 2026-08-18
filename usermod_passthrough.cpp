#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// PASSTHROUGH TEST CONFIG
// ======================================================

#define INPUT_GPIO 25

// WLED هو المسؤول عن GPIO الإخراج.

// ======================================================
// STRIP
// ======================================================

// 15 physical LEDs
// 5 WS2811 ICs
// Each IC controls 3 physical LEDs

#define WS2811_ICS 5

// 3 bytes per WS2811 IC
#define FRAME_BYTES (WS2811_ICS * 3)

// 8 bits per byte
#define FRAME_BITS (FRAME_BYTES * 8)

// ======================================================
// RMT
// ======================================================

#define RX_CHANNEL RMT_CHANNEL_0

// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns

#define RMT_CLK_DIV 2

// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ----------------------------------------------------
  // Decoder state
  // ----------------------------------------------------

  uint8_t frameBuffer[FRAME_BYTES];

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;

  uint8_t frameBytes = 0;

  bool frameReady = false;

  bool passthroughActive = false;

  // ----------------------------------------------------
  // Statistics
  // ----------------------------------------------------

  uint32_t framesReceived = 0;

  uint32_t framesDropped = 0;

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
  // RESET COMPLETE FRAME
  // ====================================================

  void discardFrame()
  {
    resetDecoder();

    frameReady = false;
  }


  // ====================================================
  // DECODE ONE SYMBOL
  // ====================================================

  bool decodeSymbol(
      const rmt_item32_t &item,
      bool &bit)
  {
    uint16_t high =
        item.duration0;

    uint16_t low =
        item.duration1;

    uint16_t total =
        high + low;


    /*
     * WS2811 @ 800 kHz
     *
     * Period ≈ 1.25 us
     *
     * RMT:
     *
     * 40 MHz
     * 1 tick = 25 ns
     *
     * Expected total ≈ 50 ticks
     *
     * We deliberately use a wide window.
     */

    if (
        total < 25 ||
        total > 80
    ) {
      return false;
    }


    /*
     * 0 = shorter HIGH
     * 1 = longer HIGH
     */

    bit = (
        high > low
    );

    return true;
  }


  // ====================================================
  // PROCESS RMT DATA
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count)
  {
    if (!items)
      return;


    /*
     * IMPORTANT:
     *
     * Never process an unlimited amount of data
     * in one WLED loop.
     *
     * Maximum symbols processed per call.
     */

    const size_t MAX_SYMBOLS = 160;

    size_t limit =
        count > MAX_SYMBOLS
        ? MAX_SYMBOLS
        : count;


    for (
        size_t i = 0;
        i < limit;
        i++
    ) {

      // ----------------------------------------------
      // If frame is already complete
      // ----------------------------------------------

      if (frameReady)
        break;


      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      // ----------------------------------------------
      // Detect long LOW / RESET
      // ----------------------------------------------

      /*
       * 50 us at 40 MHz:
       *
       * 50 us / 25 ns = 2000 ticks
       *
       * A long LOW means a new WS2811 frame.
       */

      if (
          low >= 1500 &&
          high < 100
      ) {

        /*
         * If we already have a complete frame,
         * publish it.
         */

        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        ) {

          frameReady = true;

          passthroughActive = true;

          lastFrameTime = millis();

          framesReceived++;

          break;
        }


        /*
         * Incomplete frame.
         *
         * Drop it and start again.
         */

        if (
            frameBytes > 0 ||
            bitCount > 0
        ) {

          framesDropped++;
        }


        resetDecoder();

        continue;
      }


      // ----------------------------------------------
      // Decode bit
      // ----------------------------------------------

      bool bit = false;


      if (
          !decodeSymbol(
              items[i],
              bit
          )
      ) {

        /*
         * Do NOT block WLED.
         *
         * If the symbol is bad, discard the
         * incomplete frame and resynchronize.
         */

        if (
            frameBytes > 0 ||
            bitCount > 0
        ) {

          framesDropped++;
        }


        resetDecoder();

        continue;
      }


      // ----------------------------------------------
      // Add bit
      // ----------------------------------------------

      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      // ----------------------------------------------
      // Complete byte
      // ----------------------------------------------

      if (bitCount == 8) {

        if (
            frameBytes < FRAME_BYTES
        ) {

          frameBuffer[
              frameBytes
          ] = currentByte;

          frameBytes++;
        }


        currentByte = 0;

        bitCount = 0;
      }


      // ----------------------------------------------
      // Complete frame
      // ----------------------------------------------

      if (
          frameBytes == FRAME_BYTES &&
          bitCount == 0
      ) {

        frameReady = true;

        passthroughActive = true;

        lastFrameTime = millis();

        framesReceived++;

        break;
      }
    }
  }


  // ====================================================
  // APPLY FRAME
  //
  // INPUT:
  // GRB
  //
  // REQUIRED:
  // BRG
  //
  // WLED buffer itself is RGB.
  //
  // Therefore:
  //
  // WLED R = input B
  // WLED G = input R
  // WLED B = input G
  // ====================================================

  void applyFrame()
  {
    if (!frameReady)
      return;


    for (
        uint8_t i = 0;
        i < WS2811_ICS;
        i++
    ) {

      uint8_t p =
          i * 3;


      // Incoming GRB

      uint8_t inputG =
          frameBuffer[p + 0];

      uint8_t inputR =
          frameBuffer[p + 1];

      uint8_t inputB =
          frameBuffer[p + 2];


      // GRB → BRG

      uint8_t outputR =
          inputB;

      uint8_t outputG =
          inputR;

      uint8_t outputB =
          inputG;


      /*
       * WLED receives normal RGB values.
       */

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    frameReady = false;

    resetDecoder();
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
        (gpio_num_t)
        INPUT_GPIO;


    config.clk_div =
        RMT_CLK_DIV;


    /*
     * One RMT block is enough for testing,
     * but use 2 to reduce overflow risk.
     */

    config.mem_block_num = 2;


    config.flags = 0;


    /*
     * No hardware filtering.
     */

    config.rx_config.filter_en =
        false;


    /*
     * RX chunk termination.
     *
     * This is NOT used as our frame length.
     */

    config.rx_config.idle_threshold =
        1000;


    esp_err_t err =
        rmt_config(
            &config
        );


    if (
        err != ESP_OK
    ) {

      Serial.print(
          "RMT config error: "
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
    ) {

      Serial.print(
          "RMT driver error: "
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
    ) {

      Serial.println(
          "RMT ring buffer error"
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
    ) {

      Serial.print(
          "RMT RX start error: "
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
        "Output GPIO: WLED"
    );


    Serial.println(
        "RMT: LEGACY RX"
    );


    Serial.println(
        "Protocol: WS2811"
    );


    Serial.print(
        "Physical LEDs: "
    );

    Serial.println(
        15
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
        "Speed: 800 kHz"
    );


    Serial.println(
        "Input:  GRB"
    );


    Serial.println(
        "Output: BRG"
    );


    Serial.println(
        "RMT: RX ONLY"
    );


    Serial.println(
        "Output: WLED LED ENGINE"
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
     * Extremely short operation.
     *
     * No RMT access here.
     * No blocking.
     */

    if (
        passthroughActive &&
        frameReady
    ) {

      applyFrame();
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


    /*
     * NON-BLOCKING.
     *
     * timeout = 0
     *
     * If nothing is available:
     * immediately return to WLED.
     */

    size_t receivedSize = 0;


    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items) {

      size_t count =
          receivedSize /
          sizeof(rmt_item32_t);


      processSymbols(
          items,
          count
      );


      /*
       * Always release RMT memory.
       */

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // =================================================
    // PASSTHROUGH TIMEOUT
    // =================================================

    if (
        passthroughActive
    ) {

      uint32_t now =
          millis();


      if (
          now - lastFrameTime >
          300
      ) {

        passthroughActive = false;

        frameReady = false;

        resetDecoder();


        Serial.println(
            "Passthrough signal lost"
        );


        Serial.println(
            "WLED effects resumed"
        );
      }
    }


    // =================================================
    // DEBUG
    // =================================================

    uint32_t now =
        millis();


    if (
        now - lastDebugTime >= 1000
    ) {

      lastDebugTime = now;


      Serial.print(
          "Frames: "
      );

      Serial.print(
          framesReceived
      );


      Serial.print(
          "    Dropped: "
      );

      Serial.print(
          framesDropped
      );


      Serial.print(
          "    Bytes: "
      );

      Serial.print(
          frameBytes
      );


      Serial.print(
          "    Mode: "
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


    info["physical_leds"] =
        15;


    info["ws2811_ics"] =
        WS2811_ICS;


    info["frame_bytes"] =
        FRAME_BYTES;


    info["frame_bits"] =
        FRAME_BITS;


    info["speed"] =
        "800 kHz";


    info["input_order"] =
        "GRB";


    info["output_order"] =
        "BRG";


    info["rmt"] =
        "RX ONLY";


    info["frames"] =
        framesReceived;


    info["dropped"] =
        framesDropped;
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
