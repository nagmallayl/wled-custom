#include "wled.h"
#include "driver/rmt.h"


// ======================================================
// PASSTHROUGH SETTINGS
// ======================================================

// DATA coming from LDD11
#define PASSTHROUGH_INPUT_PIN 25

// DATA going to LED strip
// CHANGE THIS if your WLED LED output uses another GPIO
#define PASSTHROUGH_OUTPUT_PIN 16


// ======================================================
// STRIP
// ======================================================

// 60 physical LEDs
// Each WS2811 IC controls 3 physical LEDs
//
// 60 / 3 = 20 ICs

#define PHYSICAL_LEDS 60
#define PIXEL_COUNT 20

// 20 IC × 3 bytes
#define FRAME_BYTES (PIXEL_COUNT * 3)

// 800 kHz WS2811
#define WS2811_KHZ 800


// ======================================================
// RMT
// ======================================================

// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns

#define RMT_CLK_DIV 2

// WS2811 reset
#define RMT_IDLE_THRESHOLD 2000

#define RMT_FILTER_THRESHOLD 1


// RX channel
#define RX_CHANNEL RMT_CHANNEL_0

// TX channel
#define TX_CHANNEL RMT_CHANNEL_1


class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;
  bool txReady = false;


  // --------------------------------------------------
  // Incoming frame
  // --------------------------------------------------

  uint8_t frameBuffer[FRAME_BYTES];

  uint16_t frameBytes = 0;


  // --------------------------------------------------
  // Statistics
  // --------------------------------------------------

  uint32_t framesReceived = 0;

  uint32_t lastDebug = 0;


  // --------------------------------------------------
  // Current byte decoder
  // --------------------------------------------------

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;


  // --------------------------------------------------
  // RMT TX buffer
  // --------------------------------------------------

  rmt_item32_t txItems[FRAME_BYTES * 8];


  // ==================================================
  // Decode WS2811 bit
  // ==================================================

  bool decodeBit(
      const rmt_item32_t &item
  ) {

    uint16_t high =
        item.duration0;

    uint16_t low =
        item.duration1;


    uint16_t total =
        high + low;


    /*
     * 800 kHz:
     *
     * 1 bit ≈ 1.25 us
     *
     * 40 MHz RMT clock:
     *
     * 1.25 us ≈ 50 ticks
     */

    if (
        total < 35 ||
        total > 65
    ) {

      return false;
    }


    /*
     * WS2811:
     *
     * longer HIGH = 1
     * shorter HIGH = 0
     */

    return high > low;
  }


  // ==================================================
  // Decode RMT RX symbols
  // ==================================================

  void decodeSymbols(
      rmt_item32_t *items,
      size_t count
  ) {

    if (!items)
      return;


    for (
        size_t i = 0;
        i < count;
        i++
    ) {

      if (
          frameBytes >= FRAME_BYTES
      ) {

        break;
      }


      rmt_item32_t &item =
          items[i];


      uint16_t high =
          item.duration0;

      uint16_t low =
          item.duration1;


      if (
          high == 0 &&
          low == 0
      ) {

        continue;
      }


      uint16_t total =
          high + low;


      /*
       * Ignore reset / invalid pulses.
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


      if (bit) {

        currentByte |= 1;
      }


      bitCount++;


      if (
          bitCount == 8
      ) {

        frameBuffer[frameBytes] =
            currentByte;


        frameBytes++;


        currentByte = 0;

        bitCount = 0;
      }
    }
  }


  // ==================================================
  // Convert one byte
  //
  // Incoming:
  //     GRB
  //
  // Current desired test:
  //     BRG
  //
  // Later we can change this very easily.
  // ==================================================

  void convertFrame() {

    for (
        uint16_t i = 0;
        i < PIXEL_COUNT;
        i++
    ) {

      uint8_t g =
          frameBuffer[i * 3 + 0];

      uint8_t r =
          frameBuffer[i * 3 + 1];

      uint8_t b =
          frameBuffer[i * 3 + 2];


      /*
       * GRB -> BRG
       */

      frameBuffer[i * 3 + 0] = b;

      frameBuffer[i * 3 + 1] = r;

      frameBuffer[i * 3 + 2] = g;
    }
  }


  // ==================================================
  // Create WS2811 RMT TX data
  // ==================================================

  uint16_t buildTxBuffer() {

    uint16_t txCount = 0;


    /*
     * 800 kHz / 40 MHz:
     *
     * 1.25 us = 50 ticks
     *
     * Typical WS2811 timing:
     *
     * 0 = HIGH ~16 ticks
     *     LOW  ~34 ticks
     *
     * 1 = HIGH ~32 ticks
     *     LOW  ~18 ticks
     */

    for (
        uint16_t byteIndex = 0;
        byteIndex < FRAME_BYTES;
        byteIndex++
    ) {

      uint8_t value =
          frameBuffer[byteIndex];


      for (
          int8_t bit = 7;
          bit >= 0;
          bit--
      ) {

        bool one =
            value & (1 << bit);


        if (one) {

          txItems[txCount].duration0 = 32;
          txItems[txCount].level0 = 1;

          txItems[txCount].duration1 = 18;
          txItems[txCount].level1 = 0;

        } else {

          txItems[txCount].duration0 = 16;
          txItems[txCount].level0 = 1;

          txItems[txCount].duration1 = 34;
          txItems[txCount].level1 = 0;
        }


        txCount++;
      }
    }


    return txCount;
  }


  // ==================================================
  // Send frame directly using RMT
  // ==================================================

  void transmitFrame() {

    if (
        frameBytes != FRAME_BYTES
    ) {

      return;
    }


    /*
     * First convert colors.
     */

    convertFrame();


    /*
     * Build raw WS2811 waveform.
     */

    uint16_t txCount =
        buildTxBuffer();


    /*
     * Send directly to GPIO.
     *
     * No:
     *
     * strip.setPixelColor()
     * strip.show()
     *
     * This is the important part
     * for reducing latency.
     */

    esp_err_t result =
        rmt_write_items(
            TX_CHANNEL,
            txItems,
            txCount,
            true
        );


    if (
        result == ESP_OK
    ) {

      framesReceived++;
    }
  }


  // ==================================================
  // Initialize RX
  // ==================================================

  bool setupRX() {

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
     * More memory for RX.
     */

    config.mem_block_num =
        8;


    config.flags = 0;


    config.rx_config.filter_en =
        true;


    config.rx_config
        .filter_ticks_thresh =
        RMT_FILTER_THRESHOLD;


    config.rx_config
        .idle_threshold =
        RMT_IDLE_THRESHOLD;


    esp_err_t result =
        rmt_config(
            &config
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX config error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    result =
        rmt_driver_install(
            RX_CHANNEL,
            8192,
            0
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX driver error: "
      );

      Serial.println(
          result
      );

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


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "RX start error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    return true;
  }


  // ==================================================
  // Initialize TX
  // ==================================================

  bool setupTX() {

    rmt_config_t config = {};


    config.rmt_mode =
        RMT_MODE_TX;


    config.channel =
        TX_CHANNEL;


    config.gpio_num =
        (gpio_num_t)
        PASSTHROUGH_OUTPUT_PIN;


    config.clk_div =
        RMT_CLK_DIV;


    config.mem_block_num =
        8;


    config.flags = 0;


    /*
     * TX configuration
     */

    config.tx_config.loop_en =
        false;


    config.tx_config.carrier_en =
        false;


    config.tx_config.idle_output_en =
        true;


    config.tx_config.idle_level =
        RMT_IDLE_LEVEL_LOW;


    esp_err_t result =
        rmt_config(
            &config
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "TX config error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    result =
        rmt_driver_install(
            TX_CHANNEL,
            0,
            0
        );


    if (
        result != ESP_OK
    ) {

      Serial.print(
          "TX driver error: "
      );

      Serial.println(
          result
      );

      return false;
    }


    return true;
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


    Serial.print(
        "Output GPIO: "
    );

    Serial.println(
        PASSTHROUGH_OUTPUT_PIN
    );


    Serial.println(
        "RMT: DIRECT RX + TX"
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
        "Input: GRB"
    );


    Serial.println(
        "Conversion: GRB -> BRG"
    );


    Serial.println(
        "================================"
    );


    // ------------------------------------------------
    // RX
    // ------------------------------------------------

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


    // ------------------------------------------------
    // TX
    // ------------------------------------------------

    if (
        setupTX()
    ) {

      txReady = true;

      Serial.println(
          "RMT TX READY"
      );

    } else {

      Serial.println(
          "RMT TX FAILED"
      );

      return;
    }


    Serial.println(
        "Passthrough READY"
    );

    Serial.println(
        "Waiting for WS2811..."
    );

    Serial.println(
        "================================"
    );
  }


  // ==================================================
  // LOOP
  // ==================================================

  void loop() override {

    if (!rxReady)
      return;


    if (!txReady)
      return;


    if (!rxRingBuffer)
      return;


    size_t receivedSize =
        0;


    /*
     * Non-blocking RX.
     */

    rmt_item32_t *items =
        (rmt_item32_t *)
        xRingbufferReceive(
            rxRingBuffer,
            &receivedSize,
            0
        );


    if (!items)
      return;


    size_t itemCount =
        receivedSize /
        sizeof(rmt_item32_t);


    /*
     * Add received symbols
     * to our 20-pixel frame.
     */

    decodeSymbols(
        items,
        itemCount
    );


    /*
     * Release RMT memory.
     */

    vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
    );


    /*
     * We received exactly:
     *
     * 20 IC × 3 bytes
     *
     * = 60 bytes
     */

    if (
        frameBytes >= FRAME_BYTES
    ) {

      transmitFrame();


      /*
       * Prepare immediately
       * for next frame.
       */

      frameBytes = 0;

      currentByte = 0;

      bitCount = 0;
    }


    /*
     * Debug only once per second.
     */

    uint32_t now =
        millis();


    if (
        now - lastDebug >= 1000
    ) {

      lastDebug = now;


      Serial.print(
          "Frames: "
      );

      Serial.println(
          framesReceived
      );
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


    info["output"] =
        PASSTHROUGH_OUTPUT_PIN;


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


// ======================================================
// REGISTER
// ======================================================

static PassthroughUsermod
passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
