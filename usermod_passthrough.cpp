#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIGURATION
// ======================================================

#define PASSTHROUGH_INPUT_PIN 25

// WLED output GPIO
// WLED itself controls this pin.
#define PASSTHROUGH_OUTPUT_PIN 16

// ======================================================
// WS2811
// ======================================================

// 60 physical LEDs
// Each WS2811 IC controls 3 physical LEDs
// 60 / 3 = 20 ICs
#define PHYSICAL_LEDS 60
#define PIXEL_COUNT   20

// 3 bytes per WS2811 IC
#define FRAME_BYTES   (PIXEL_COUNT * 3)

// WS2811 speed
#define WS2811_KHZ 800

// ======================================================
// RMT
// ======================================================

// ESP32 classic RMT:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2

// 50 us
// 50 us / 25 ns = 2000 ticks
#define RMT_IDLE_THRESHOLD 2000

// RX channel
#define RX_CHANNEL RMT_CHANNEL_0

// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ====================================================
  // FRAME BUFFER
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  uint16_t frameBytes = 0;

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;

  // A complete frame is waiting for WLED
  volatile bool frameReady = false;

  // Passthrough active
  bool passthroughActive = false;

  // Statistics
  uint32_t framesReceived = 0;
  uint32_t framesDropped = 0;

  uint32_t lastFrameTime = 0;
  uint32_t lastDebug = 0;


  // ====================================================
  // RESET DECODER
  // ====================================================

  void resetDecoder()
  {
    frameBytes = 0;
    currentByte = 0;
    bitCount = 0;
  }


  // ====================================================
  // RESYNCHRONIZE
  // ====================================================

  void resynchronize()
  {
    resetDecoder();
  }


  // ====================================================
  // DECODE ONE WS2811 BIT
  // ====================================================

  bool decodeBit(const rmt_item32_t &item)
  {
    uint16_t high =
        item.duration0;

    uint16_t low =
        item.duration1;

    uint16_t total =
        high + low;


    /*
     * 800 kHz WS2811:
     *
     * Total ≈ 50 ticks
     *
     * 0:
     * HIGH ≈ 16
     * LOW  ≈ 34
     *
     * 1:
     * HIGH ≈ 32
     * LOW  ≈ 18
     */

    if (total < 35 || total > 65)
      return false;


    return high > low;
  }


  // ====================================================
  // PROCESS RMT SYMBOLS
  // ====================================================

  void processSymbols(
      rmt_item32_t *items,
      size_t count)
  {
    if (!items)
      return;


    for (size_t i = 0;
         i < count;
         i++) {

      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      if (high == 0 && low == 0)
        continue;


      /*
       * ------------------------------------------------
       * WS2811 RESET / IDLE
       * ------------------------------------------------
       *
       * With:
       *
       * 40 MHz RMT clock
       * 1 tick = 25 ns
       *
       * 2000 ticks = 50 us
       *
       * A LOW period >= 50 us means:
       *
       * NEW FRAME
       *
       * This is also our resynchronization point.
       */

      if (low >= RMT_IDLE_THRESHOLD) {

        /*
         * If we have exactly one complete frame,
         * mark it ready.
         */

        if (
            frameBytes == FRAME_BYTES &&
            bitCount == 0
        ) {

          frameReady = true;

          passthroughActive = true;

          lastFrameTime = millis();

          framesReceived++;

        }
        else if (frameBytes != 0) {

          /*
           * Incomplete / corrupted frame.
           *
           * Drop it and resynchronize.
           */

          framesDropped++;
        }


        /*
         * Always start cleanly after RESET.
         */

        resetDecoder();

        continue;
      }


      /*
       * ------------------------------------------------
       * INVALID SYMBOL
       * ------------------------------------------------
       */

      uint16_t total =
          high + low;


      if (total < 35 || total > 65) {

        /*
         * A bad symbol can destroy bit alignment.
         *
         * Do NOT continue decoding from the
         * middle of the corrupted frame.
         */

        resynchronize();

        continue;
      }


      /*
       * ------------------------------------------------
       * PROTECT AGAINST EXTRA DATA
       * ------------------------------------------------
       */

      if (frameBytes >= FRAME_BYTES) {

        resynchronize();

        continue;
      }


      /*
       * ------------------------------------------------
       * DECODE BIT
       * ------------------------------------------------
       */

      bool bit =
          decodeBit(items[i]);


      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      /*
       * ------------------------------------------------
       * COMPLETE BYTE
       * ------------------------------------------------
       */

      if (bitCount == 8) {

        frameBuffer[frameBytes] =
            currentByte;

        frameBytes++;

        currentByte = 0;

        bitCount = 0;


        /*
         * We do NOT immediately output here.
         *
         * We wait for the WS2811 RESET/IDLE.
         *
         * This prevents partially received frames
         * from being displayed.
         */
      }
    }
  }


  // ====================================================
  // COLOR CONVERSION
  //
  // INPUT:
  //
  //     G R B
  //
  // REQUIRED OUTPUT:
  //
  //     B R G
  //
  // WLED buffer uses:
  //
  //     R G B
  //
  // Therefore the RGB values placed into WLED are:
  //
  //     R = input B
  //     G = input R
  //     B = input G
  //
  // This produces:
  //
  //     B R G
  //
  // when WLED outputs RGB.
  // ====================================================

  void applyFrameToWLED()
  {
    if (!frameReady)
      return;


    for (
        uint16_t i = 0;
        i < PIXEL_COUNT;
        i++
    ) {

      uint16_t p =
          i * 3;


      // Incoming GRB
      uint8_t inputG =
          frameBuffer[p + 0];

      uint8_t inputR =
          frameBuffer[p + 1];

      uint8_t inputB =
          frameBuffer[p + 2];


      // =================================================
      // GRB -> BRG
      // =================================================

      uint8_t outputR =
          inputB;

      uint8_t outputG =
          inputR;

      uint8_t outputB =
          inputG;


      /*
       * Put the converted RGB values
       * into WLED's LED buffer.
       *
       * WLED performs the actual output.
       */

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    frameReady = false;
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
        PASSTHROUGH_INPUT_PIN;


    config.clk_div =
        RMT_CLK_DIV;


    /*
     * Four RMT memory blocks.
     */

    config.mem_block_num = 4;


    config.flags = 0;


    /*
     * No filtering.
     *
     * We want to preserve the original WS2811
     * timing as much as possible.
     */

    config.rx_config.filter_en =
        false;


    /*
     * 50 us idle / reset.
     */

    config.rx_config.idle_threshold =
        RMT_IDLE_THRESHOLD;


    esp_err_t result =
        rmt_config(&config);


    if (result != ESP_OK) {

      Serial.print(
          "RX config error: "
      );

      Serial.println(result);

      return false;
    }


    result =
        rmt_driver_install(
            RX_CHANNEL,
            8192,
            0
        );


    if (result != ESP_OK) {

      Serial.print(
          "RX driver error: "
      );

      Serial.println(result);

      return false;
    }


    result =
        rmt_get_ringbuf_handle(
            RX_CHANNEL,
            &rxRingBuffer
        );


    if (
        result != ESP_OK ||
        rxRingBuffer == nullptr
    ) {

      Serial.println(
          "RX ring buffer error"
      );

      return false;
    }


    result =
        rmt_rx_start(
            RX_CHANNEL,
            true
        );


    if (result != ESP_OK) {

      Serial.print(
          "RX start error: "
      );

      Serial.println(result);

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
        PASSTHROUGH_INPUT_PIN,
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
        PASSTHROUGH_INPUT_PIN
    );


    Serial.print(
        "Output GPIO: "
    );

    Serial.println(
        PASSTHROUGH_OUTPUT_PIN
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
        PHYSICAL_LEDS
    );


    Serial.print(
        "WS2811 ICs: "
    );

    Serial.println(
        PIXEL_COUNT
    );


    Serial.print(
        "Frame bytes: "
    );

    Serial.println(
        FRAME_BYTES
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


    if (setupRX()) {

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
  //
  // This runs before WLED sends the LEDs.
  //
  // We overwrite the LED buffer with the
  // latest received frame.
  //
  // NO strip.show() here.
  // ====================================================

  void handleOverlayDraw() override
  {
    if (!passthroughActive)
      return;


    applyFrameToWLED();
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
      return;


    size_t receivedSize = 0;


    /*
     * NON-BLOCKING RX
     *
     * This is important for minimizing latency.
     */

    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (items) {

      size_t itemCount =
          receivedSize /
          sizeof(rmt_item32_t);


      processSymbols(
          items,
          itemCount
      );


      /*
       * Release RMT memory immediately.
       */

      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // =================================================
    // AUTO PASSTHROUGH TIMEOUT
    // =================================================

    if (passthroughActive) {

      uint32_t now =
          millis();


      /*
       * If no frame arrived for 300 ms,
       * return control to WLED.
       */

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
        now - lastDebug >= 1000
    ) {

      lastDebug = now;


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
          "    Mode: "
      );


      if (passthroughActive) {

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
        PASSTHROUGH_INPUT_PIN;


    info["output"] =
        PASSTHROUGH_OUTPUT_PIN;


    info["physical_leds"] =
        PHYSICAL_LEDS;


    info["ws2811_ics"] =
        PIXEL_COUNT;


    info["frame_bytes"] =
        FRAME_BYTES;


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
