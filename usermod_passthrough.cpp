#include "wled.h"
#include "driver/rmt.h"

// ======================================================
// PINS
// ======================================================

#define PASSTHROUGH_INPUT_PIN  25
#define PASSTHROUGH_OUTPUT_PIN 16

// ======================================================
// WS2811
// ======================================================

// 60 physical LEDs
// Each WS2811 IC controls 3 LEDs
// 60 / 3 = 20 ICs

#define PHYSICAL_LEDS 60
#define PIXEL_COUNT   20

// 3 bytes per WS2811 IC
#define FRAME_BYTES   (PIXEL_COUNT * 3)

// 800 kHz
#define WS2811_KHZ    800


// ======================================================
// RMT
// ======================================================

// ESP32 classic RMT clock:
// 80 MHz / 2 = 40 MHz
// 1 tick = 25 ns

#define RMT_CLK_DIV 2

// WS2811 reset / end of frame
#define RMT_IDLE_THRESHOLD 2000

// Ignore pulses shorter than this
#define RMT_FILTER_THRESHOLD 1


// ======================================================
// RMT CHANNELS
// ======================================================

// RX = channel 0
#define RX_CHANNEL RMT_CHANNEL_0

// TX = channel 1
#define TX_CHANNEL RMT_CHANNEL_1


// ======================================================
// RMT MEMORY
// ======================================================

// ESP32 classic has 8 RMT memory blocks total.
//
// OLD:
// RX = 8
// TX = 8
//
// This caused:
// TX config error: 258
//
// NEW:
// RX = 4
// TX = 4

#define RX_MEM_BLOCKS 4
#define TX_MEM_BLOCKS 4


// ======================================================
// USERMOD
// ======================================================

class PassthroughUsermod : public Usermod {

private:

  RingbufHandle_t rxRingBuffer = nullptr;

  bool rxReady = false;
  bool txReady = false;


  // ====================================================
  // Frame buffer
  // ====================================================

  uint8_t frameBuffer[FRAME_BYTES];

  uint16_t frameBytes = 0;

  uint8_t currentByte = 0;

  uint8_t bitCount = 0;


  // ====================================================
  // TX buffer
  // ====================================================

  rmt_item32_t txItems[FRAME_BYTES * 8];


  // ====================================================
  // Statistics
  // ====================================================

  uint32_t framesReceived = 0;

  uint32_t lastDebug = 0;


  // ====================================================
  // Decode one WS2811 bit
  // ====================================================

  bool decodeBit(const rmt_item32_t &item)
  {
    uint16_t high = item.duration0;
    uint16_t low  = item.duration1;

    uint16_t total = high + low;

    // 800 kHz:
    // approximately 50 RMT ticks per bit

    if (total < 35 || total > 65)
      return false;

    // Long HIGH = 1
    // Short HIGH = 0

    return high > low;
  }


  // ====================================================
  // Decode received RMT symbols
  // ====================================================

  void decodeSymbols(
      rmt_item32_t *items,
      size_t count)
  {
    if (!items)
      return;


    for (size_t i = 0; i < count; i++) {

      if (frameBytes >= FRAME_BYTES)
        break;


      uint16_t high =
          items[i].duration0;

      uint16_t low =
          items[i].duration1;


      if (high == 0 && low == 0)
        continue;


      uint16_t total =
          high + low;


      /*
       * Ignore reset / invalid pulses.
       */

      if (total < 35 || total > 65)
        continue;


      bool bit =
          decodeBit(items[i]);


      currentByte <<= 1;


      if (bit)
        currentByte |= 1;


      bitCount++;


      if (bitCount == 8) {

        frameBuffer[frameBytes] =
            currentByte;

        frameBytes++;

        currentByte = 0;

        bitCount = 0;
      }
    }
  }


  // ====================================================
  // COLOR CONVERSION
  //
  // Incoming:
  //     GRB
  //
  // Output:
  //     BRG
  //
  // G R B
  // ↓ ↓ ↓
  // B R G
  // ====================================================

  void convertFrame()
  {
    for (uint16_t i = 0;
         i < PIXEL_COUNT;
         i++) {

      uint8_t g =
          frameBuffer[i * 3 + 0];

      uint8_t r =
          frameBuffer[i * 3 + 1];

      uint8_t b =
          frameBuffer[i * 3 + 2];


      // GRB -> BRG

      frameBuffer[i * 3 + 0] = b;

      frameBuffer[i * 3 + 1] = r;

      frameBuffer[i * 3 + 2] = g;
    }
  }


  // ====================================================
  // Build WS2811 waveform
  // ====================================================

  uint16_t buildTxBuffer()
  {
    uint16_t txCount = 0;


    /*
     * 40 MHz RMT clock
     *
     * 800 kHz WS2811
     *
     * 1 bit = 50 ticks
     *
     * 0:
     * HIGH = 16
     * LOW  = 34
     *
     * 1:
     * HIGH = 32
     * LOW  = 18
     */


    for (uint16_t byteIndex = 0;
         byteIndex < FRAME_BYTES;
         byteIndex++) {

      uint8_t value =
          frameBuffer[byteIndex];


      for (int8_t bit = 7;
           bit >= 0;
           bit--) {

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


  // ====================================================
  // Transmit frame
  // ====================================================

  void transmitFrame()
  {
    if (frameBytes != FRAME_BYTES)
      return;


    // Color conversion

    convertFrame();


    // Build waveform

    uint16_t txCount =
        buildTxBuffer();


    /*
     * Send using RMT TX.
     *
     * rmt_write_items() handles
     * the data using the configured
     * RMT memory.
     */

    esp_err_t result =
        rmt_write_items(
            TX_CHANNEL,
            txItems,
            txCount,
            true
        );


    if (result == ESP_OK) {

      framesReceived++;
    }
  }


  // ====================================================
  // RX SETUP
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
     * IMPORTANT:
     *
     * 4 blocks instead of 8.
     *
     * The RX ring buffer will
     * provide the frame in chunks.
     */

    config.mem_block_num =
        RX_MEM_BLOCKS;


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


  // ====================================================
  // TX SETUP
  // ====================================================

  bool setupTX()
  {
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


    /*
     * IMPORTANT:
     *
     * 4 blocks instead of 8.
     *
     * RX = 4
     * TX = 4
     *
     * Total = 8
     */

    config.mem_block_num =
        TX_MEM_BLOCKS;


    config.flags = 0;


    config.tx_config.loop_en =
        false;


    config.tx_config.carrier_en =
        false;


    config.tx_config.idle_output_en =
        true;


    config.tx_config.idle_level =
        RMT_IDLE_LEVEL_LOW;


    esp_err_t result =
        rmt_config(&config);


    if (result != ESP_OK) {

      Serial.print(
          "TX config error: "
      );

      Serial.println(result);

      return false;
    }


    result =
        rmt_driver_install(
            TX_CHANNEL,
            0,
            0
        );


    if (result != ESP_OK) {

      Serial.print(
          "TX driver error: "
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
        "RMT: LEGACY DIRECT RX + TX"
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


    Serial.print(
        "RX blocks: "
    );

    Serial.println(
        RX_MEM_BLOCKS
    );


    Serial.print(
        "TX blocks: "
    );

    Serial.println(
        TX_MEM_BLOCKS
    );


    Serial.println(
        "================================"
    );


    // RX

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


    // TX

    if (setupTX()) {

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
    if (!rxReady)
      return;


    if (!txReady)
      return;


    if (!rxRingBuffer)
      return;


    size_t receivedSize = 0;


    /*
     * NON-BLOCKING.
     *
     * Important for minimizing delay.
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
     * Add this chunk to
     * the current frame.
     */

    decodeSymbols(
        items,
        itemCount
    );


    /*
     * Release RX memory
     * immediately.
     */

    vRingbufferReturnItem(
        rxRingBuffer,
        (void *)items
    );


    /*
     * 60 bytes received.
     */

    if (
        frameBytes >= FRAME_BYTES
    ) {

      transmitFrame();


      /*
       * Immediately prepare
       * for the next frame.
       */

      frameBytes = 0;

      currentByte = 0;

      bitCount = 0;
    }


    /*
     * Serial debug only once
     * every second.
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


    info["speed"] =
        "800 kHz";


    info["input_order"] =
        "GRB";


    info["output_order"] =
        "BRG";


    info["rx_blocks"] =
        RX_MEM_BLOCKS;


    info["tx_blocks"] =
        TX_MEM_BLOCKS;


    info["frames"] =
        framesReceived;
  }


  uint16_t getId() override
  {
    return 0x5041;
  }
};


// ======================================================
// REGISTER USERMOD
// ======================================================

static PassthroughUsermod
passthroughUsermod;


REGISTER_USERMOD(
    passthroughUsermod
);
