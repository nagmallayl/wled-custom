#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// CONFIGURATION
// ======================================================

#define PASSTHROUGH_INPUT_PIN 25

// لا نحدد GPIO للإخراج هنا.
// WLED هو المسؤول عن GPIO الخاص بالشريط.

// ======================================================
// WS2811 CONFIG
// ======================================================

// الشريط يحتوي 60 LED فعلي
// كل WS2811 IC يتحكم في 3 LED
// 60 / 3 = 20 IC

#define PHYSICAL_LEDS 60
#define WS2811_ICS    20

// كل IC يستقبل 3 بايت
#define FRAME_BYTES   (WS2811_ICS * 3)

// كل بايت = 8 bits
// 20 IC × 24 bits = 480 symbols
#define FRAME_BITS    (FRAME_BYTES * 8)

// 800 kHz
#define WS2811_KHZ    800

// ======================================================
// RMT
// ======================================================

// ESP32 classic
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns

#define RMT_CLK_DIV 2

#define RX_CHANNEL RMT_CHANNEL_0

// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;

  // ----------------------------------------------------
  // Incoming frame
  // ----------------------------------------------------

  uint8_t frameBuffer[FRAME_BYTES];

  uint16_t frameBytes = 0;
  uint8_t currentByte = 0;
  uint8_t bitCount = 0;

  volatile bool frameReady = false;

  bool passthroughActive = false;

  // ----------------------------------------------------
  // Statistics
  // ----------------------------------------------------

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
  // DECODE WS2811 BIT
  // ====================================================

  bool decodeBit(const rmt_item32_t &item)
  {
    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total = high + low;

    /*
     * 800 kHz WS2811
     *
     * Total period ≈ 1.25 us
     *
     * With 40 MHz RMT:
     *
     * 1 tick = 25 ns
     *
     * Total ≈ 50 ticks
     */

    if (total < 35 || total > 65)
      return false;


    /*
     * WS2811:
     *
     * 0 = shorter HIGH
     * 1 = longer HIGH
     */

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


    for (size_t i = 0; i < count; i++) {

      uint16_t high = items[i].duration0;
      uint16_t low  = items[i].duration1;

      uint16_t total = high + low;


      // ------------------------------------------------
      // Empty symbol
      // ------------------------------------------------

      if (high == 0 && low == 0)
        continue;


      // ------------------------------------------------
      // If frame is already complete
      //
      // Ignore remaining symbols from this RX block.
      // ------------------------------------------------

      if (frameReady)
        continue;


      // ------------------------------------------------
      // Invalid timing
      // ------------------------------------------------

      if (total < 35 || total > 65) {

        /*
         * Bad timing.
         *
         * The important difference from the old version:
         *
         * We completely reset the decoder instead of
         * continuing with a potentially incorrect bit
         * alignment.
         */

        if (frameBytes > 0 || bitCount > 0)
          framesDropped++;

        resetDecoder();

        continue;
      }


      // ------------------------------------------------
      // Decode bit
      // ------------------------------------------------

      bool bit = decodeBit(items[i]);


      currentByte <<= 1;

      if (bit)
        currentByte |= 1;


      bitCount++;


      // ------------------------------------------------
      // Complete byte
      // ------------------------------------------------

      if (bitCount == 8) {

        if (frameBytes < FRAME_BYTES) {

          frameBuffer[frameBytes] =
              currentByte;

          frameBytes++;
        }

        currentByte = 0;
        bitCount = 0;
      }


      // ------------------------------------------------
      // COMPLETE FRAME
      // ------------------------------------------------

      if (frameBytes == FRAME_BYTES &&
          bitCount == 0) {

        /*
         * 20 IC × 24 bits
         *
         * = 480 bits
         *
         * = 60 bytes
         */

        frameReady = true;

        passthroughActive = true;

        lastFrameTime = millis();

        framesReceived++;

        /*
         * Do not reset here.
         *
         * We wait until applyFrameToWLED()
         * consumes the frame.
         */

        break;
      }
    }
  }


  // ====================================================
  // APPLY FRAME
  //
  // Incoming:
  //
  //     GRB
  //
  // Required strip order:
  //
  //     BRG
  //
  // WLED buffer itself is RGB.
  //
  // Therefore:
  //
  // WLED R = incoming B
  // WLED G = incoming R
  // WLED B = incoming G
  //
  // ====================================================

  void applyFrameToWLED()
  {
    if (!frameReady)
      return;


    for (
        uint16_t i = 0;
        i < WS2811_ICS;
        i++
    ) {

      uint16_t p = i * 3;


      // Incoming GRB

      uint8_t inputG =
          frameBuffer[p + 0];

      uint8_t inputR =
          frameBuffer[p + 1];

      uint8_t inputB =
          frameBuffer[p + 2];


      // ------------------------------------------------
      // GRB → BRG
      // ------------------------------------------------

      uint8_t outputR =
          inputB;

      uint8_t outputG =
          inputR;

      uint8_t outputB =
          inputG;


      // ------------------------------------------------
      // Write to WLED buffer
      // ------------------------------------------------

      strip.setPixelColor(
          i,
          outputR,
          outputG,
          outputB
      );
    }


    /*
     * Frame consumed.
     */

    frameReady = false;

    resetDecoder();
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
        (gpio_num_t)
        PASSTHROUGH_INPUT_PIN;


    config.clk_div =
        RMT_CLK_DIV;


    /*
     * Four memory blocks.
     *
     * 256 RMT symbols internally.
     * Data continues through the ring buffer.
     */

    config.mem_block_num = 4;


    config.flags = 0;


    /*
     * No filter.
     *
     * We want the original WS2811 timing.
     */

    config.rx_config.filter_en =
        false;


    /*
     * The receiver may terminate a chunk after
     * a sufficiently long idle period.
     *
     * We no longer depend on this to detect
     * the frame.
     *
     * Frame length itself determines completion.
     */

    config.rx_config.idle_threshold =
        1000;


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
     * NON-BLOCKING.
     *
     * This is important because we do not want
     * the Usermod to hold WLED.
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


      vRingbufferReturnItem(
          rxRingBuffer,
          (void *)items
      );
    }


    // =================================================
    // SIGNAL TIMEOUT
    // =================================================

    if (passthroughActive) {

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
          "    Bytes: "
      );

      Serial.print(
          frameBytes
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
        PASSTHROUGH_INPUT_PIN;


    info["output"] =
        "WLED";


    info["physical_leds"] =
        PHYSICAL_LEDS;


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
// REGISTER
// ======================================================

static PassthroughUsermod passthroughUsermod;

REGISTER_USERMOD(
    passthroughUsermod
);
