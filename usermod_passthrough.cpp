#include "wled.h"
#include "driver/rmt.h"

#define PASSTHROUGH_INPUT_PIN 25
#define PASSTHROUGH_RMT_CHANNEL RMT_CHANNEL_0

// ==================================================
// STRIP SETTINGS
// ==================================================

// 60 physical LEDs
// Each WS2811 IC controls 3 LEDs
#define PHYSICAL_LEDS 60

// 60 / 3 = 20 WS2811 ICs
#define PIXEL_COUNT 20

// 3 bytes per WS2811 IC
#define FRAME_BYTES (PIXEL_COUNT * 3)

// WS2811 data speed
#define WS2811_KHZ 800


// ==================================================
// RMT SETTINGS
// ==================================================

// ESP32 RMT clock:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns
#define RMT_CLK_DIV 2

// WS2811 reset time
#define RMT_IDLE_THRESHOLD 2000

// Ignore very short pulses
#define RMT_FILTER_THRESHOLD 4


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rmtRingBuffer = nullptr;

  bool rmtReady = false;

  // 20 IC × 3 bytes
  // = 60 bytes
  uint8_t frameBuffer[FRAME_BYTES];

  uint16_t frameBytes = 0;

  uint32_t framesReceived = 0;

  uint32_t lastDebug = 0;


  // ==================================================
  // Decode one WS2811 bit
  // ==================================================

  bool decodeBit(
      const rmt_item32_t &item
  ) {

    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total = high + low;

    /*
     * 800 kHz WS2811:
     *
     * 1 bit ≈ 1.25 us
     *
     * RMT:
     *
     * 1 tick = 25 ns
     *
     * 1.25 us ≈ 50 ticks
     */

    if (total < 35 || total > 65)
      return false;

    /*
     * Longer HIGH = logic 1
     * Shorter HIGH = logic 0
     */

    return high > low;
  }


  // ==================================================
  // Decode RMT symbols
  // ==================================================

  void decodeSymbols(
      rmt_item32_t *items,
      size_t itemCount
  ) {

    if (!items)
      return;


    /*
     * These need to survive between
     * RMT chunks because one WS2811
     * frame may be split across chunks.
     */

    static uint8_t currentByte = 0;
    static uint8_t bitCount = 0;


    for (
        size_t i = 0;
        i < itemCount &&
        frameBytes < FRAME_BYTES;
        i++
    ) {

      rmt_item32_t &item = items[i];

      uint16_t high = item.duration0;
      uint16_t low  = item.duration1;


      if (high == 0 && low == 0)
        continue;


      uint16_t total =
          high + low;


      /*
       * Only accept WS2811 800 kHz bits.
       */

      if (
          total < 35 ||
          total > 65
      ) {
        continue;
      }


      bool bit =
          decodeBit(item);


      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      /*
       * Complete byte.
       */

      if (bitCount == 8) {

        frameBuffer[frameBytes] =
            currentByte;

        frameBytes++;

        currentByte = 0;

        bitCount = 0;
      }
    }
  }


  // ==================================================
  // Send complete frame
  //
  // Incoming:
  //     GRB
  //
  // Required:
  //     BRG
  // ==================================================

  void sendFrame() {

    if (
        frameBytes != FRAME_BYTES
    )
      return;


    /*
     * We have:
     *
     * 20 ICs
     *
     * 60 bytes
     */

    for (
        uint16_t i = 0;
        i < PIXEL_COUNT;
        i++
    ) {

      /*
       * Incoming order:
       *
       * byte 0 = G
       * byte 1 = R
       * byte 2 = B
       */

      uint8_t g =
          frameBuffer[i * 3 + 0];

      uint8_t r =
          frameBuffer[i * 3 + 1];

      uint8_t b =
          frameBuffer[i * 3 + 2];


      /*
       * Required output:
       *
       * B R G
       *
       * WLED setPixelColor()
       * receives RGB.
       *
       * Therefore:
       *
       * R = B
       * G = R
       * B = G
       */

      strip.setPixelColor(
          i,
          b,
          r,
          g
      );
    }


    /*
     * Send the 20-pixel frame.
     *
     * Each pixel represents
     * 3 physical LEDs on the strip.
     */

    strip.show();


    framesReceived++;


    /*
     * Debug once per second.
     */

    uint32_t now =
        millis();


    if (
        now - lastDebug >= 1000
    ) {

      lastDebug = now;


      Serial.println();

      Serial.println(
          "----- Passthrough RX -----"
      );


      Serial.print(
          "Frames: "
      );

      Serial.println(
          framesReceived
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
          "Bytes: "
      );

      Serial.println(
          FRAME_BYTES
      );


      Serial.println(
          "Speed: 800 kHz"
      );


      Serial.println(
          "Conversion: GRB -> BRG"
      );


      Serial.println(
          "--------------------------"
      );
    }
  }


public:


  // ==================================================
  // SETUP
  // ==================================================

  void setup() override {

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
        "================================"
    );


    // ==================================================
    // RMT CONFIGURATION
    // ==================================================

    rmt_config_t config = {};


    config.rmt_mode =
        RMT_MODE_RX;


    config.channel =
        PASSTHROUGH_RMT_CHANNEL;


    config.gpio_num =
        (gpio_num_t)
        PASSTHROUGH_INPUT_PIN;


    config.clk_div =
        RMT_CLK_DIV;


    /*
     * Use multiple RMT memory blocks.
     */

    config.mem_block_num = 8;


    config.flags = 0;


    /*
     * Input filter.
     */

    config.rx_config.filter_en =
        true;


    config.rx_config
        .filter_ticks_thresh =
        RMT_FILTER_THRESHOLD;


    /*
     * Detect WS2811 reset.
     */

    config.rx_config
        .idle_threshold =
        RMT_IDLE_THRESHOLD;


    // ==================================================
    // INSTALL RMT DRIVER
    // ==================================================

    esp_err_t result =
        rmt_config(
            &config
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "ERROR rmt_config: "
      );

      Serial.println(
          result
      );

      return;
    }


    result =
        rmt_driver_install(
            PASSTHROUGH_RMT_CHANNEL,
            8192,
            0
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "ERROR rmt_driver_install: "
      );

      Serial.println(
          result
      );

      return;
    }


    // ==================================================
    // GET RING BUFFER
    // ==================================================

    result =
        rmt_get_ringbuf_handle(
            PASSTHROUGH_RMT_CHANNEL,
            &rmtRingBuffer
        );


    if (
        result != ESP_OK ||
        rmtRingBuffer == nullptr
    ) {

      Serial.println(
          "ERROR: RMT ring buffer"
      );

      return;
    }


    // ==================================================
    // START RMT RX
    // ==================================================

    result =
        rmt_rx_start(
            PASSTHROUGH_RMT_CHANNEL,
            true
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "ERROR rmt_rx_start: "
      );

      Serial.println(
          result
      );

      return;
    }


    rmtReady = true;


    Serial.println(
        "RMT RX READY"
    );


    Serial.println(
        "Waiting for WS2811 DATA on GPIO25..."
    );


    Serial.println(
        "================================"
    );
  }


  // ==================================================
  // LOOP
  // ==================================================

  void loop() override {

    if (!rmtReady)
      return;


    if (!rmtRingBuffer)
      return;


    size_t receivedSize = 0;


    /*
     * Non-blocking receive.
     *
     * Important for reducing delay.
     */

    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rmtRingBuffer,
            &receivedSize,
            0
        );


    if (!items)
      return;


    size_t itemCount =
        receivedSize /
        sizeof(rmt_item32_t);


    /*
     * Decode this chunk.
     */

    decodeSymbols(
        items,
        itemCount
    );


    /*
     * Return RMT memory.
     */

    vRingbufferReturnItem(
        rmtRingBuffer,
        (void *)items
    );


    /*
     * We expect:
     *
     * 20 IC × 3 bytes
     *
     * = 60 bytes
     */

    if (
        frameBytes >= FRAME_BYTES
    ) {

      sendFrame();


      /*
       * Prepare for next frame.
       */

      frameBytes = 0;
    }
  }


  // ==================================================
  // WLED INFO
  // ==================================================

  void addToJsonInfo(
      JsonObject &root
  ) override {

    JsonObject info =
        root["u"]
        .createNestedObject(
            "Passthrough"
        );


    info["input"] =
        PASSTHROUGH_INPUT_PIN;


    info["physical_leds"] =
        PHYSICAL_LEDS;


    info["ws2811_ics"] =
        PIXEL_COUNT;


    info["speed"] =
        "800 kHz";


    info["conversion"] =
        "GRB -> BRG";


    info["frames"] =
        framesReceived;
  }


  uint16_t getId() override {

    return 0x5041;
  }
};


// ==================================================
// REGISTER USERMOD
// ==================================================

static PassthroughUsermod
passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
